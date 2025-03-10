package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.LoggedTunableNumber;
import frc.robot.Subsystems.BeamBreak.BeamBreakIO;
import frc.robot.Subsystems.BeamBreak.BeamBreakIOInputsAutoLogged;
import frc.robot.Subsystems.Dealgae.Dealgae;
import frc.robot.Subsystems.Dealgae.DealgaeIO;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorIO;
import frc.robot.Subsystems.Funnel.Funnel;
import frc.robot.Subsystems.Funnel.FunnelIO;
import frc.robot.Subsystems.LEDs.LEDs;
import frc.robot.Subsystems.LEDs.LEDs.LEDStates;

public class Superstructure extends SubsystemBase {
    private Dealgae s_dealgae;
    private Elevator s_elevator;
    private EndEffector s_endeffector;
    private LEDs led;
    private BeamBreakIO beambreak;
    private Funnel s_funnel;
    private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

    private double stateStartTime = 0;
    private SuperstructureStates systemState = SuperstructureStates.IDLE;

    LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("Superstructure/Intake POST BEAMBREAK VOLTAGE", 1);
    LoggedTunableNumber intakeTime = new LoggedTunableNumber("Superstructure/Intake POST BEAMBREAK TIME", 0.5);
    LoggedTunableNumber scoreVoltage = new LoggedTunableNumber("Superstructure/Score PRE BEAMBREAK Voltage", 1);
    LoggedTunableNumber scoreTime = new LoggedTunableNumber("Superstructure/Score PRE BEAMBREAK Time", 0.5);

    public Superstructure(DealgaeIO dealgaeIO, ElevatorIO elevatorIO, EndEffectorIO endEffectorIO, LEDs led, FunnelIO funnelIO, BeamBreakIO beamBreakIO){
        this.s_dealgae = new Dealgae(dealgaeIO);
        this.s_elevator = new Elevator(elevatorIO);
        this.s_endeffector = new EndEffector(endEffectorIO);
        this.s_funnel = new Funnel(funnelIO);
        this.beambreak = beamBreakIO;
        this.led = led;
    }

    public enum SuperstructureStates {
        IDLE,
        INTAKE_A,
        INTAKE_B,
        POST_INTAKE,
        SCORE_A,
        SCORE_B,
        DEALGAE_A,
        DEALGAE_B,
        DEALGAED,
        PROCESSOR,
        ELEVATOR_DOWN
    }

    @Override
    public void periodic(){
        s_dealgae.Loop();
        s_elevator.Loop();
        s_endeffector.Loop();
        s_funnel.Loop();
        led.Loop();
        beambreak.updateInputs(beamBreakInputs);
        Logger.processInputs("BeamBreak", beamBreakInputs);
        Logger.recordOutput("SuperstructureState", this.systemState);
        Logger.recordOutput("State start time", stateStartTime);
        switch(systemState){
            case IDLE:
                if (DriverStation.isDisabled()) {
                    led.setState(LEDStates.DISABLED);
                } else {
                    if (s_elevator.selectedHeight == "L2"){
                        led.requestL2();
                    } if (s_elevator.selectedHeight == "L3"){
                        led.requestL3();
                    } if (s_elevator.selectedHeight == "L4"){
                        led.requestL4();
                    } if (s_elevator.selectedHeight == "L1"){
                        led.requestIdleLED();
                    }
                }
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIdle();
                s_endeffector.requestIdle();
                break;
            case INTAKE_A:
                led.requestFunnelIntakingLED();
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIntake(3);
                s_endeffector.requestIntake(4);
                if (RobotController.getFPGATime() / 1.056 - stateStartTime > 0.5 && isBeamBroken()){
                    setState(SuperstructureStates.INTAKE_B);
                }
                break;
            case INTAKE_B:
                led.requestFunnelIntakingLED();
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIntake(2);
                s_endeffector.requestIntake(intakeVoltage.get());
                if (RobotController.getFPGATime() / 1.056 - stateStartTime > intakeTime.get()){
                    setState(SuperstructureStates.POST_INTAKE);
                }
                break;
            case POST_INTAKE:
                led.requestFunnelIntookLED();
                s_dealgae.requestIdle();
                s_funnel.requestIdle();
                s_elevator.requestIdle();
                s_endeffector.requestIdle();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SCORE_A:
                led.requestScoringLED();
                s_dealgae.requestIdle();
                s_elevator.requestMotionMagicCoral();
                s_funnel.requestIdle();
                s_endeffector.requestIdle();
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.SCORE_B);
                }
                break;
            case SCORE_B:
                led.requestScoringLED();
                s_dealgae.requestIdle();
                s_elevator.requestHold();
                s_endeffector.requestScore(4);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.ELEVATOR_DOWN);
                }
                break;
            case DEALGAE_A:
                led.requestDealgaingLED();
                s_dealgae.requestIdle();
                s_elevator.requestMotionMagicAlgae();
                s_endeffector.requestIdle();
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.DEALGAE_B);
                }
                break;
            case DEALGAE_B:
                led.requestDealgaingLED();
                s_dealgae.requestDealgae(3);
                s_elevator.requestHold();
                s_endeffector.requestIdle();
                if (s_dealgae.getDealgaeCurrent() > 27 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5){
                    setState(SuperstructureStates.DEALGAED);
                }
                break;
            case DEALGAED:
                led.requestDealgaedLED();
                s_dealgae.requestIdle();
                s_elevator.requestElevatorDown();
                s_endeffector.requestIdle();
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case PROCESSOR:
                led.requestProcessingLED();
                s_dealgae.requestProcessor(-3);
                s_elevator.requestElevatorDown();
                s_endeffector.requestIdle();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;            
            case ELEVATOR_DOWN:
                if (s_elevator.selectedHeight == "L2"){
                    led.requestL2();
                } if (s_elevator.selectedHeight == "L3"){
                    led.requestL3();
                } if (s_elevator.selectedHeight == "L4"){
                    led.requestL4();
                } if (s_elevator.selectedHeight == "L1"){
                    led.requestIdleLED();
                }   
                s_dealgae.requestIdle();
                s_elevator.requestElevatorDown();
                s_endeffector.requestIdle();
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(SuperstructureStates.IDLE);
    }

    public void requestScore(){
        setState(SuperstructureStates.SCORE_A);
    }

    public void requestIntake(){
        setState(SuperstructureStates.INTAKE_A);
    }

    public void requestDealgae(){
        setState(SuperstructureStates.DEALGAE_A);
    }

    public void requestProcessor(){
        setState(SuperstructureStates.PROCESSOR);
    }

    public void requestElevatorDown(){
        setState(SuperstructureStates.ELEVATOR_DOWN);
    }

    public void setL1(){
        s_elevator.setHeight("L1");
    }

    public void setL2(){
        s_elevator.setHeight("L2");
    }

    public void setL3(){
        s_elevator.setHeight("L3");
    }

    public void setL4(){
        s_elevator.setHeight("L4");
    }


    public void executePurple(){
        if(s_elevator.selectedHeight == "L1"){
            requestIntake();
        }
        else{
            requestScore();
        }
    }

    public void executeGreen(){
        if(s_elevator.selectedHeight == "L1"){
            requestProcessor();
        }
        else{
            requestDealgae();
        }
    }

    public boolean isBeamBroken(){
        return beamBreakInputs.beamBroken;
    }

    public void setState(SuperstructureStates nextState){
        systemState = nextState;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}