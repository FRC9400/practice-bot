package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private double stateStartTime = 0;
    private SuperstructureStates systemState = SuperstructureStates.IDLE;

    public Superstructure(ElevatorIO elevatorIO, EndEffectorIO endEffectorIO, IntakeIO intakeIO){
        this.s_elevator = new Elevator(elevatorIO);
        this.s_endeffector = new EndEffector(endEffectorIO);
        this.s_intake = new Intake(intakeIO);
    }

    public enum SuperstructureStates{
        //To-do
        IDLE
    }

    @Override
    public void periodic(){
        s_elevator.Loop();
        s_endeffector.Loop();
        s_intake.Loop();
        Logger.recordOutput("SuperstructureState", this.systemState);
        Logger.recordOutput("State start time", stateStartTime);
        switch(systemState){
            case IDLE:
                s_elevator.requestIdle();
                s_intake.requestIdle();
                s_endeffector.requestIdle();
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(SuperstructureStates.IDLE);
    }

     public void setState(SuperstructureStates nextState){
        systemState = nextState;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}
