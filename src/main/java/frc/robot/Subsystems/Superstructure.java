package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.LoggedTunableNumber;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Amp.Amp;
import frc.robot.Subsystems.Amp.AmpIO;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerIO;
import frc.robot.Subsystems.OTB_Intake.OTB_Intake;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIO;
import frc.robot.Subsystems.Shooter.ShooterArm;
import frc.robot.Subsystems.Shooter.ShooterArmIO;


public class Superstructure extends SubsystemBase{
    private Indexer s_indexer;
    private OTB_Intake s_intake;
    private ShooterArm s_shooter;
    private Amp s_amp;
    private double stateStartTime = 0;

    private SuperstructureStates systemState = SuperstructureStates.IDLE;
    
    
    LoggedTunableNumber shooterMidAngle = new LoggedTunableNumber("Superstructure/shootMidAngle", 23);
    LoggedTunableNumber shootMidVel = new LoggedTunableNumber("Superstructure/shootMidvel", 70);
    LoggedTunableNumber passAngle = new LoggedTunableNumber("Superstructure/passAngle", 23);
    LoggedTunableNumber passVel = new LoggedTunableNumber("Superstructure/passVel", 70);

    LoggedTunableNumber autosVel = new LoggedTunableNumber("Autos/velocity", 40);
    LoggedTunableNumber autosRatio = new LoggedTunableNumber("Autos/ratio", 0.6);

    private double[] shooterVelocity = {0,0}; //left vel + ratio
    private double armAngleDegrees = 0;

    public Superstructure(IndexerIO indexer, OTB_IntakeIO intake, ShooterArmIO shooter, AmpIO amp){
        this.s_indexer = new Indexer(indexer);
        this.s_intake = new OTB_Intake(intake);
        this.s_shooter = new ShooterArm(shooter);
        this.s_amp = new Amp(amp);
    }

    public enum SuperstructureStates{
        IDLE,
        INTAKE,
        NOTE,
        OUTTAKE,
        HANDOFF_A,
        HANDOFF_B,
        AMP_A,
        AMP_B,
        BOWEN,
        SHOOT,
        PASS,
        PRE_PASS,
        PREPARE_SHOOT,
        POST_SHOOT,
        AUTO_INTAKE,
        AUTO_HANDOFF,
        AUTO_POST_INDEX,
        PREPARE_SHOOT_AUTO,
        SHOOT_AUTO,
        POST_SHOOT_AUTO
    }

    @Override
    public void periodic(){
        s_indexer.Loop();
        s_intake.Loop();
        s_shooter.Loop();
        s_amp.Loop();
        Logger.recordOutput("SuperstructureState", this.systemState);
        Logger.recordOutput("State start time", stateStartTime);
        switch(systemState){
            case IDLE:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestZero();
                RobotContainer.controller.setRumble(RumbleType.kLeftRumble, 0);
                break;
            case INTAKE:
                s_indexer.requestHandoff(2);
                s_amp.requestRun(2);
                s_intake.requestIntake();
                s_shooter.requestZero();
                if (s_amp.getAmpCurrent() > 20 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25) {
                    setState(SuperstructureStates.HANDOFF_A);
                }
                break;
            case NOTE:
                RobotContainer.controller.setRumble(RumbleType.kLeftRumble, 1);
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestZero();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.05) {
                    RobotContainer.controller.setRumble(RumbleType.kLeftRumble, 0);
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case OUTTAKE:
                s_indexer.requestHandoff(-3);
                s_amp.requestRun(-3);
                s_intake.requestOuttake();
                s_shooter.requestIdle();
                break;
            case HANDOFF_A:
                s_indexer.requestHandoff(2);//placeholder value(s)
                s_amp.requestRun(2);
                s_intake.requestSetpoint();
                s_shooter.requestIdle();
                if (s_amp.getAmpCurrent() < 20 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.25) {
                    setState(SuperstructureStates.HANDOFF_B);
                } //TEMP
                break;
            case HANDOFF_B:
                s_indexer.requestIdle();//placeholder value(s)
                s_amp.requestRun(-1);
                s_intake.requestSetpoint();
                s_shooter.requestIdle();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.35) {
                    setState(SuperstructureStates.NOTE);
                }
                break;
            case AMP_A:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestAmp();
                break;
            case AMP_B:
                s_indexer.requestIdle();
                s_amp.requestRun(-3);
                s_intake.requestSetpoint();
                s_shooter.requestAmp();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.75) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case BOWEN:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestShoot(shootMidVel.get(), 0.5,shooterMidAngle.get());
                break;
            case PREPARE_SHOOT:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestShoot(shooterVelocity[0], shooterVelocity[1], armAngleDegrees);
                if(s_shooter.atShooterSetpoint() && s_shooter.atArmSetpoint()){
                    setState(SuperstructureStates.SHOOT);
                }
                break;
            case SHOOT:
                s_indexer.requestIdle();
                s_amp.requestRun(3.6);
                s_intake.requestSetpoint();
                s_shooter.requestShoot(shooterVelocity[0], shooterVelocity[1], armAngleDegrees);//placeholder value(s)
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.POST_SHOOT);
                }
                break;
            case POST_SHOOT:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestZero();
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.03) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case PRE_PASS:
            
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestSetpoint();
                s_shooter.requestShoot(passVel.get(), 1, passAngle.get());//placeholder value(s)
                if(s_shooter.atShooterSetpoint() && s_shooter.atArmSetpoint()){
                    setState(SuperstructureStates.PASS);
                }
                break;
            case PASS:
                s_indexer.requestIdle();
                s_amp.requestRun(2);
                s_intake.requestSetpoint();
                s_shooter.requestShoot(passVel.get(), 1,passAngle.get());//placeholder value(s)
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case AUTO_INTAKE:
                s_indexer.requestHandoff(2);
                s_amp.requestRun(2);
                s_intake.requestIntake();
                s_shooter.requestAutos(autosVel.get(), autosRatio.get());
                if (s_amp.getAmpCurrent() > 25 && RobotController.getFPGATime() / 1.0E6 -stateStartTime > 0.25){
                    setState(SuperstructureStates.AUTO_HANDOFF);
                }
                break;
            case AUTO_HANDOFF:
                s_indexer.requestHandoff(3);
                s_amp.requestRun(3);
                s_intake.requestDown();
                s_shooter.requestAutos(autosVel.get(), autosRatio.get());
                if (RobotController.getFPGATime() / 1.0E6 -stateStartTime > 0.37){
                    setState(SuperstructureStates.AUTO_POST_INDEX);
                }
                break;
            case AUTO_POST_INDEX:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestDown();
                s_shooter.requestAutos(autosVel.get(), autosRatio.get());
                break;
            case PREPARE_SHOOT_AUTO:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestDown();
                s_shooter.requestShoot(shooterVelocity[0], shooterVelocity[1], armAngleDegrees);
                if(s_shooter.atShooterSetpoint() && s_shooter.atArmSetpoint()){
                    setState(SuperstructureStates.SHOOT_AUTO);
                }
                break;
            case SHOOT_AUTO:
                s_indexer.requestIdle();
                s_amp.requestRun(3.6); 
                s_intake.requestDown();
                s_shooter.requestShoot(shooterVelocity[0], shooterVelocity[1], armAngleDegrees);
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.6) {
                    setState(SuperstructureStates.POST_SHOOT_AUTO);
                }
                break;
            case POST_SHOOT_AUTO:
                s_indexer.requestIdle();
                s_amp.requestIdle();
                s_intake.requestDown();
                s_shooter.requestAutos(autosVel.get(), autosRatio.get());
                break;
            }
        }

    public void requestIdle(){
        setState(SuperstructureStates.IDLE);
    }

    public void requestAmp(){
        setState(SuperstructureStates.AMP_A);
    }

    public void requestIntake(){
        setState(SuperstructureStates.INTAKE);
    }

    public void requestAutoIntake(){
        setState(SuperstructureStates.AUTO_INTAKE);
    }

    public void requestAutoPreShoot(double vel, double ratio, double angle){
        shooterVelocity[0] = vel;
        shooterVelocity[1] = ratio;
        armAngleDegrees = angle;
        setState(SuperstructureStates.PREPARE_SHOOT_AUTO);
    }

    public void requestPreShoot(double velocity, double ratio, double angleDeg){
        shooterVelocity[0] = velocity;
        shooterVelocity[1] = ratio;
        armAngleDegrees = angleDeg;
        setState(SuperstructureStates.PREPARE_SHOOT);
    }

    public void requestPass(){
        setState(SuperstructureStates.PASS);
    }    

    public void requestLowPass(){
        shooterVelocity[0] = 40;
        shooterVelocity[1] = 1;
        armAngleDegrees = 0;
        setState(SuperstructureStates.PREPARE_SHOOT);
    }

    public void setState(SuperstructureStates next){
        systemState=next;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}