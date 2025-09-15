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
    private String elevatorSetpoint = "L1";
    private SuperstructureStates systemState = SuperstructureStates.IDLE;

    public Superstructure(ElevatorIO elevatorIO, EndEffectorIO endEffectorIO, IntakeIO intakeIO, BeamBreakIO beamBreakIO){
        this.s_elevator = new Elevator(elevatorIO);
        this.s_endeffector = new EndEffector(endEffectorIO);
        this.s_intake = new Intake(intakeIO);
        this.beambreak = beamBreakIO;
    }

    public enum SuperstructureStates{
        //To-do
        IDLE,
        INTAKE,
        SCORE_A,
        SCORE_B,
        ELEVATOR_DOWN
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
                s_elevator.requestL1();
                s_intake.requestIntake(3);
                s_endeffector.requestIntake(4);
                if (isBeamBroken()){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case SCORE_A:
                if (elevatorSetpoint == "L1"){
                    s_elevator.requestL1();
                } else if (elevatorSetpoint == "L2"){
                    s_elevator.requestL2();
                } else if (elevatorSetpoint == "L3"){
                    s_elevator.requestL3();
                } else if (elevatorSetpoint == "L4"){
                    s_elevator.requestL4();
                }
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.SCORE_B);
                }
                break;
            case SCORE_B:
                s_elevator.requestHold();
                s_endeffector.requestScore(4);
                s_intake.requestIdle();
                if (!isBeamBroken()){
                    setState(SuperstructureStates.ELEVATOR_DOWN);
                }
                break;
            case ELEVATOR_DOWN:
                s_elevator.requestL1();
                s_endeffector.requestIdle();
                s_intake.requestIdle();
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

    public void requestIntake(){
        setState(SuperstructureStates.INTAKE);
    }

    public void requestScoreL1(){
        elevatorSetpoint = "L1";
        setState(SuperstructureStates.SCORE_A);
    }

    public void requestScoreL2(){
        elevatorSetpoint = "L2";
        setState(SuperstructureStates.SCORE_A);
    }

    public void requestScoreL3(){
        elevatorSetpoint = "L3";
        setState(SuperstructureStates.SCORE_A);
    }

    public void requestScoreL4(){
        elevatorSetpoint = "L4";
        setState(SuperstructureStates.SCORE_A);
    }

    public void requestElevatorDown(){
        elevatorSetpoint = "L1";
        setState(SuperstructureStates.ELEVATOR_DOWN);
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
