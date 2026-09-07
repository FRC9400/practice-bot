package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.pivotConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Subsystems.BeamBreak.BeamBreakIO;
import frc.robot.Subsystems.BeamBreak.BeamBreakIOInputsAutoLogged;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorIO;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Shooter.ShooterIO;
import frc.robot.Subsystems.Pivot.Pivot;
import frc.robot.Subsystems.Pivot.PivotIO;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private Shooter s_shooter;
    private Pivot s_pivot;
    private Elevator s_elevator;
    private EndEffector s_endeffector;
    private Intake s_intake;
    private BeamBreakIO beambreak;
    private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();


    private double stateStartTime = 0;
    private SuperstructureStates systemState = SuperstructureStates.ZERO;
    private double pivotSetpoint = pivotConstants.stowAngleDegrees;

    private final double zeroDuration = 0.25;
    private final double feedDuration = 0.5;

    private final double intakeVolts = 6;
    private final double feedVolts = 8;

    public Superstructure(ShooterIO shooterIO, PivotIO pivotIO, IntakeIO intakeIO, BeamBreakIO beamBreakIO){
        this.s_shooter = new Shooter(shooterIO);
        this.s_pivot = new Pivot(pivotIO);
        this.s_intake = new Intake(intakeIO);
        this.beambreak = beamBreakIO;
    }

    public enum SuperstructureStates{
        IDLE,
        ZERO,
        INTAKE,
        AIM,
        READY,
        SHOOT
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
                s_shooter.requestIdle();
                s_pivot.requestStow();
                s_intake.requestIdle();
                break;
            case ZERO:
                s_shooter.requestIdle();
                s_pivot.zeroSensor();
                s_intake.requestIdle();
                if (timeInState()>zeroDuration){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case INTAKE:
                s_shooter.requestIdle();
                s_pivot.requestStow();
                s_intake.requestIntake(intakeVolts);
                if (isBeamBroken()){
                    setState(SuperstructureStates.AIM);
                }
                break;

            case AIM:
                s_shooter.requestMMVelocity(shooterConstants.shootSpeedMPS);
                s_pivot.requestAngle(pivotSetpoint);
                s_intake.requestIdle();
                if (s_pivot.atSetpoint()&&s_shooter.atSetpoint()){
                    setState(SuperstructureStates.READY);
                }
                break;
            case READY:
                s_shooter.requestMMVelocity(shooterConstants.shootSpeedMPS);
                s_pivot.requestAngle(pivotSetpoint);
                s_intake.requestIdle();
                break;
            case SHOOT:
                s_shooter.requestMMVelocity(shooterConstants.shootSpeedMPS);
                s_pivot.requestAngle(pivotSetpoint);
                s_intake.requestIntake(feedVolts);
                if (timeInState()>feedDuration){
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

    public void requestZero(){
        setState(SuperstructureStates.ZERO);
    }

    public void requestIntake(){
        setState(SuperstructureStates.INTAKE);
    }

    public void requestShoot(){
        setState(SuperstructureStates.SHOOT);
    }

    public void requestAimSubwoofer(){
        pivotSetpoint = pivotConstants.subwooferAngleDegrees;
        setState(SuperstructureStates.AIM);
    }

    public void requestAimPodium(){
        pivotSetpoint = pivotConstants.podiumAngleDegrees;
        setState(SuperstructureStates.AIM);
    }

    public boolean isBeamBroken(){
        return beamBreakInputs.beamBroken;
    }

    public double timeInState(){
        return (RobotController.getFPGATime() / 1E6) - stateStartTime;
    }

     public void setState(SuperstructureStates nextState){
        systemState = nextState;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}
