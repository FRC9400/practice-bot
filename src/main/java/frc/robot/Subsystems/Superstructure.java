package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;
import frc.robot.Subsystems.BeamBreak.BeamBreakIO;
import frc.robot.Subsystems.BeamBreak.BeamBreakIO.BeamBreakIOInputs;
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
    private double elevatorSetpoint = 0;
    
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
        LEVEL,
        SCORE,
        ELEVATORDOWN
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
                s_elevator.requestIdle();
                s_intake.requestIntake(6);
                s_endeffector.requestIntake(7);

                if(isBeamBroken() == true){
                    setState(SuperstructureStates.IDLE);
                }
                
                break;
            
            case LEVEL:
                s_intake.requestIdle();
                s_endeffector.requestIdle();

                if(elevatorSetpoint == elevatorConstants.L1){
                    s_elevator.requestL1();
                } else if(elevatorSetpoint == elevatorConstants.L2){
                    s_elevator.requestL2();
                } else if (elevatorSetpoint == elevatorConstants.L3){
                    s_elevator.requestL3();
                } else if (elevatorSetpoint == elevatorConstants.L4){
                    s_elevator.requestL4();
                }

                if(s_elevator.atSetpoint() == true){
                    setState(SuperstructureStates.SCORE);
                }

                break;

            case SCORE:
                s_elevator.requestHold();
                s_intake.requestIdle();
                s_endeffector.requestScore(8);
                
                if(isBeamBroken() == true){
                    setState(SuperstructureStates.IDLE);
                }
                break;

            case ELEVATORDOWN:
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                s_elevator.requestL1();

                if(s_elevator.atSetpoint() == true){
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

    public void requestL1(){
        elevatorSetpoint = elevatorConstants.L1;
        setState(SuperstructureStates.LEVEL);
    }

    public void requestL2(){
        elevatorSetpoint = elevatorConstants.L2;
        setState(SuperstructureStates.LEVEL);
    }

    public void requestL3(){
        elevatorSetpoint = elevatorConstants.L3;
        setState(SuperstructureStates.LEVEL);
    }

    public void requestL4(){
        elevatorSetpoint = elevatorConstants.L4;
        setState(SuperstructureStates.LEVEL);
    }

    public void requestScore(){
        setState(SuperstructureStates.SCORE);
    }

    public void requestElevatorDown(){
        setState(SuperstructureStates.ELEVATORDOWN);
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
