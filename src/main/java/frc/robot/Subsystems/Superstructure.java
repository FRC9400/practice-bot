package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.LoggedTunableNumber;
import frc.robot.Subsystems.Dealgae.Dealgae;
import frc.robot.Subsystems.Dealgae.DealgaeIO;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorIO;
import frc.robot.Subsystems.LEDs.LEDs;
import frc.robot.Subsystems.LEDs.LEDs.LEDStates;

public class Superstructure extends SubsystemBase {
    private Dealgae s_dealgae;
    private Elevator s_elevator;
    private EndEffector s_endeffector;
    private LEDs led;

    private double stateStartTime = 0;
    private SuperstructureStates systemState = SuperstructureStates.IDLE;
    private String selectedHeight = "L1";

    LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("Superstructure/End Effector Intake Voltage", 3);
    LoggedTunableNumber intakeCurrentLimit = new LoggedTunableNumber("Superstructure/End Effector Intake Current Lower Limit", 20);
    LoggedTunableNumber scoreVoltage = new LoggedTunableNumber("Superstructure/End Effector Score Voltage", -3);
    LoggedTunableNumber scoreSeconds = new LoggedTunableNumber("Superstructure/End Effector Score Seconds", 2);
    LoggedTunableNumber dealgaeVoltage = new LoggedTunableNumber("Superstructure/Dealgae Dealgae Voltage", 3);
    LoggedTunableNumber dealgaeCurrent = new LoggedTunableNumber("Superstructure/Dealgae Dealgae Current", 25);
    LoggedTunableNumber processorVoltage = new LoggedTunableNumber("Superstructure/Dealgae Processor Voltage", -3);
    LoggedTunableNumber processorSeconds = new LoggedTunableNumber("Superstructure/Dealgae Processor Seconds", 2);


    public Superstructure(DealgaeIO dealgaeIO, ElevatorIO elevatorIO, EndEffectorIO endEffectorIO, LEDs led){
        this.s_dealgae = new Dealgae(dealgaeIO);
        this.s_elevator = new Elevator(elevatorIO);
        this.s_endeffector = new EndEffector(endEffectorIO);
        this.led = led;
    }

    public enum SuperstructureStates {
        IDLE,
        INTAKE,
        POST_INTAKE,
        SCORE_A,
        SCORE_B,
        DEALGAE_A,
        DEALGAE_B,
        PROCESSOR,
        POST_PROCESSOR,
        SET_HEIGHT,
        ELEVATOR_DOWN
    }

    @Override
    public void periodic(){
        s_dealgae.Loop();
        s_elevator.Loop();
        s_endeffector.Loop();
        led.Loop();
        Logger.recordOutput("SuperstructureState", this.systemState);
        Logger.recordOutput("State start time", stateStartTime);
        switch(systemState){
            case IDLE:
                if (DriverStation.isDisabled()) {
                    led.setState(LEDStates.DISABLED);
                }
                else {
                    led.setState(LEDStates.IDLE);
                }
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_endeffector.requestIdle();
                break;
            case INTAKE:
                led.requestIntakingLED();
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_endeffector.requestIntake(intakeVoltage.get());
                if (s_endeffector.getEndEffectorCurrent() > 15 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1){
                    setState(SuperstructureStates.POST_INTAKE);
                }
                break;
            case POST_INTAKE:
                led.requestIntookLED();
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_endeffector.requestIdle();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SCORE_A:
                led.requestScoringLED();
                s_dealgae.requestIdle();
                s_elevator.requestMotionMagic();
                s_endeffector.requestIdle();
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.SCORE_B);
                }
                break;
            case SCORE_B:
                led.requestScoringLED();
                s_dealgae.requestIdle();
                s_elevator.requestHold();
                s_endeffector.requestScore(scoreVoltage.get());
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > scoreSeconds.get()) {
                    setState(SuperstructureStates.ELEVATOR_DOWN);
                }
                break;
            case DEALGAE_A:
                led.requestDealgaingLED();
                s_dealgae.requestIdle();
                s_elevator.requestMotionMagic();
                s_endeffector.requestIdle();
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.DEALGAE_B);
                }
                break;
            case DEALGAE_B:
                led.requestDealgaingLED();
                s_dealgae.requestDealgae(dealgaeVoltage.get());
                s_elevator.requestHold();
                s_endeffector.requestIdle();
                if (s_dealgae.getDealgaeCurrent() > dealgaeCurrent.get() && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5){
                    setState(SuperstructureStates.ELEVATOR_DOWN);
                }
                break;
            case PROCESSOR:
                led.requestProcessingLED();
                s_dealgae.requestProcessor(processorVoltage.get());
                s_elevator.requestElevatorDown();
                s_endeffector.requestIdle();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > processorSeconds.get()) {
                    setState(SuperstructureStates.POST_PROCESSOR);
                }
                break;
            case POST_PROCESSOR:
                led.requestProcessedLED();
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_endeffector.requestIdle();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SET_HEIGHT:
                led.requestHeightSetLED();
                s_dealgae.requestIdle();
                s_elevator.setHeight(selectedHeight);
                s_endeffector.requestIdle();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;                
            case ELEVATOR_DOWN:
                led.requestScoredLED();
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
        setState(SuperstructureStates.INTAKE);
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
        selectedHeight = "L1";
        setState(SuperstructureStates.SET_HEIGHT);
    }

    public void setL2(){
        selectedHeight = "L2";
        setState(SuperstructureStates.SET_HEIGHT);
    }

    public void setL3(){
        selectedHeight = "L3";
        setState(SuperstructureStates.SET_HEIGHT);
    }

    public void setL4(){
        selectedHeight = "L4";
        setState(SuperstructureStates.SET_HEIGHT);
    }

    public void setState(SuperstructureStates nextState){
        systemState = nextState;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}