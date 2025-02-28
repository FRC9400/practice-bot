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
import frc.robot.Subsystems.LEDs.LEDs;
import frc.robot.Subsystems.LEDs.LEDs.LEDStates;

public class Superstructure extends SubsystemBase {
    private Dealgae s_dealgae;
    private Elevator s_elevator;
    private EndEffector s_endeffector;
    private LEDs led;

    private double stateStartTime = 0;
    private SuperstructureStates systemState = SuperstructureStates.IDLE;

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
        INTAKE2,
        OUTTAKE,
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
                s_endeffector.requestIntake(3);
                if (s_endeffector.getEndEffectorCurrent() > 11 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1){
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
            case INTAKE2:
                led.requestIntakingLED();
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_endeffector.requestIntake(3);
                if (RobotController.getFPGATime() / 1.056 - stateStartTime > 3){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case OUTTAKE:
                led.requestIntakingLED();
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_endeffector.requestIntake(-4);
                if (RobotController.getFPGATime() / 1.056 - stateStartTime > 4){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SCORE_A:
                led.requestScoringLED();
                s_dealgae.requestIdle();
                s_elevator.requestMotionMagicCoral();
                s_endeffector.requestIdle();
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.SCORE_B);
                }
                break;
            case SCORE_B:
                led.requestScoringLED();
                s_dealgae.requestIdle();
                s_elevator.requestHold();
                s_endeffector.requestScore(3);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 2) {
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
                led.requestIdleLED();
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

    public void requestIntake2(){
        setState(SuperstructureStates.INTAKE2);
    }

    public void requestOuttake(){
        setState(SuperstructureStates.OUTTAKE);
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

    public void setState(SuperstructureStates nextState){
        systemState = nextState;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}