package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.BeamBreak.BeamBreakIO;
import frc.robot.Subsystems.BeamBreak.BeamBreakIOInputsAutoLogged;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorIO;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private Elevator s_elevator;
    private EndEffector s_endeffector;
    private Intake s_intake;
    private BeamBreakIO beambreak;
    private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();


    private double stateStartTime = 0;
    private SuperstructureStates systemState = SuperstructureStates.IDLE;

    public Superstructure(ElevatorIO elevatorIO, EndEffectorIO endEffectorIO, IntakeIO intakeIO, BeamBreakIO beamBreakIO){
        this.s_elevator = new Elevator(elevatorIO);
        this.s_endeffector = new EndEffector(endEffectorIO);
        this.s_intake = new Intake(intakeIO);
        this.beambreak = beamBreakIO;
    }

    public enum SuperstructureStates{
        IDLE,
        INTAKE,
        SCORE,
        ELEVATORL1,
        ELEVATORL2,
        ELEVATORL3,
        ELEVATORL4,
        SCORE_L1,
        SCORE_L2,
        SCORE_L3,
        SCORE_L4,
    }

    @Override
    public void periodic(){
        s_elevator.Loop();
        s_endeffector.Loop();
        s_intake.Loop();
        beambreak.updateInputs(beamBreakInputs);
        Logger.processInputs("BeamBreak", beamBreakInputs);
        Logger.recordOutput("SuperstructureState", this.systemState);
        Logger.recordOutput("State start time", stateStartTime);
        switch(systemState){
            case IDLE:
                s_elevator.requestIdle();
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                break;
            case INTAKE:
                s_intake.requestIntake(3);
                s_endeffector.requestIntake(3);
                s_elevator.requestIdle();
                if (isBeamBroken()) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SCORE:
                s_endeffector.requestScore(3);
                s_intake.requestIdle();
                s_elevator.requestIdle();
                if (!isBeamBroken()) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case ELEVATORL1:
                s_elevator.requestL1();
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                break;
            case ELEVATORL2:
                s_elevator.requestL2();
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                break;
            case ELEVATORL3:
                s_elevator.requestL3();
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                break;
            case ELEVATORL4:
                s_elevator.requestL4();
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                break;
            case SCORE_L1:
                s_elevator.requestL1();
                if (s_elevator.atSetpoint()) {
                    s_endeffector.requestScore(3);
                    if (!isBeamBroken()) {
                        setState(SuperstructureStates.IDLE);
                    }
                }
                break;
            case SCORE_L2:
                s_elevator.requestL2();
                if (s_elevator.atSetpoint()) {
                    s_endeffector.requestScore(3);
                    if (!isBeamBroken()) {
                        setState(SuperstructureStates.IDLE);
                    }
                }
                break;
            case SCORE_L3:
                s_elevator.requestL3();
                if (s_elevator.atSetpoint()) {
                    s_endeffector.requestScore(3);
                    if (!isBeamBroken()) {
                        setState(SuperstructureStates.IDLE);
                    }
                }
                break;
            case SCORE_L4:
                s_elevator.requestL4();
                if (s_elevator.atSetpoint()) {
                    s_endeffector.requestScore(3);
                    if (!isBeamBroken()) {
                        setState(SuperstructureStates.IDLE);
                    }
                }
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(SuperstructureStates.IDLE);
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

    public void requestIntake() {
        setState(SuperstructureStates.INTAKE);
    }

    public void requestScore() {
        setState(SuperstructureStates.SCORE);
    }

    public void requestScoreL1() {
        setState(SuperstructureStates.SCORE_L1);
    }

    public void requestScoreL2() {
        setState(SuperstructureStates.SCORE_L2);
    }

    public void requestScoreL3() {
        setState(SuperstructureStates.SCORE_L3);
    }

    public void requestScoreL4() {
        setState(SuperstructureStates.SCORE_L4);
    }
}
