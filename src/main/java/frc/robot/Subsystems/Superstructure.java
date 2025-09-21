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
        OUTTAKE,
        L1,
        L2,
        L3,
        L4,
        SCORE,
        LOWER
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
                s_endeffector.requestIntake(3);
                if(isBeamBroken()){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case OUTTAKE:
                s_elevator.requestL1();
                s_intake.requestIntake(-5);
                s_endeffector.requestIntake(-5);
                break;
            case L1:
                s_elevator.requestL1();
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                if(s_elevator.atSetpoint()){
                    setState(SuperstructureStates.SCORE);
                }
                break;
            case L2:
                s_elevator.requestL2();
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                if(s_elevator.atSetpoint()){
                    setState(SuperstructureStates.SCORE);
                }
                break;
            case L3:
                s_elevator.requestL3();
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                if(s_elevator.atSetpoint()){
                    setState(SuperstructureStates.SCORE);
                }
                break;
            case L4:
                s_elevator.requestL4();
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                if(s_elevator.atSetpoint()){
                    setState(SuperstructureStates.SCORE);
                }
                break;
            case SCORE:
                s_elevator.requestHold();
                s_intake.requestIntake(4);
                s_endeffector.requestScore(4);
                if(RobotController.getFPGATime() * 1E6 - stateStartTime > 5 && !isBeamBroken()){
                    requestLower();
                }
                break;
            case LOWER:
                s_elevator.requestL1();
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                if(s_elevator.atSetpoint()){
                    requestIdle();
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

    public void requestOuttake(){
        setState(SuperstructureStates.OUTTAKE);
    }

    public void requestL1(){
        setState(SuperstructureStates.L1);
    }

    public void requestL2(){
        setState(SuperstructureStates.L2);
    }

    public void requestL3(){
        setState(SuperstructureStates.L3);
    }

    public void requestL4(){
        setState(SuperstructureStates.L4);
    }

    // public void requestScore(){
    //     setState(SuperstructureStates.SCORE);
    // }

    public void requestLower(){
        setState(SuperstructureStates.LOWER);
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
