package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

public class ShooterArm{
    private final ShooterArmIO shooterArmIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private ShooterArmStates shooterArmState = ShooterArmStates.IDLE;
    private double[] shooterVelocity = {0,0}; //left vel + ratio
    private double armAngleDegrees = 0;

    public enum ShooterArmStates{
        IDLE,
        ZERO,
        HANDOFF,
        SHOOT,
        AMP,
        PASS,
        VOLTAGE,
        OUTTAKE,
        VELOCITY
    }

    public ShooterArm(ShooterArmIO shooterArmIO) {
        this.shooterArmIO = shooterArmIO;
    }

    
    public void Loop(){
        shooterArmIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("ShooterArmState", this.shooterArmState);
        switch(shooterArmState){
            case IDLE:
                shooterArmIO.requestArmVoltage(0);
                shooterArmIO.requestShooterVoltage(0);
                break;
            case ZERO:
                shooterArmIO.requestMotionMagicSetpoint(0);
                shooterArmIO.requestVelocity(0.0, 0.5);
                break;
            case HANDOFF:
                 shooterArmIO.requestMotionMagicSetpoint(0);
                 shooterArmIO.requestVelocity(shooterVelocity[0], shooterVelocity[1]);
                 break;
            case SHOOT:
                shooterArmIO.requestVelocity(shooterVelocity[0], shooterVelocity[1]);
                shooterArmIO.requestMotionMagicSetpoint(armAngleDegrees);
                break;
            case AMP:
                shooterArmIO.requestVelocity(0, 0);
                shooterArmIO.requestMotionMagicSetpoint(130);
                break;
            case PASS:
                shooterArmIO.requestVelocity(shooterVelocity[0], shooterVelocity[1]);
                shooterArmIO.requestPositionSetpoint(armAngleDegrees);
                break;
            case VOLTAGE:
                shooterArmIO.requestShooterVoltage(4);
                shooterArmIO.requestArmVoltage(0);
            case OUTTAKE:
                shooterArmIO.requestShooterVoltage(-1.2);
                shooterArmIO.requestArmVoltage(0);
            case VELOCITY:
                shooterArmIO.requestMotionMagicSetpoint(0);
                shooterArmIO.requestVelocity(shooterVelocity[0], shooterVelocity[1]);
            default:
                break;

        }
    }

    public void requestIdle(){
        setState(shooterArmState.IDLE);
    }

    public void requestShoot(double velocityRPS, double ratio, double armAngleDegrees){
        shooterVelocity[0] = velocityRPS;
        shooterVelocity[1] = ratio;
        this.armAngleDegrees = armAngleDegrees;
        setState(shooterArmState.SHOOT);
    }

    public void requestAmp(){
        setState(shooterArmState.AMP);
    }

    public void requestOuttake(){
        setState(shooterArmState.OUTTAKE);
    }

    public void requestZero(){
        setState(ShooterArmStates.ZERO);
    }

    public void requestHandoff(double velocityRPS, double ratio){
        shooterVelocity[0] = velocityRPS;
        shooterVelocity[1] = ratio;
        setState(ShooterArmStates.HANDOFF);
    }
    
    public void requestPass(double velocityRPS, double ratio, double armAngleDeg){
        shooterVelocity[0] = velocityRPS;
        shooterVelocity[1] = ratio;
        this.armAngleDegrees = armAngleDegrees;
        setState(shooterArmState.PASS);
    }

    public void requestVoltage(){
        setState(shooterArmState.VOLTAGE);
    }

    public void requestAutos(double vel, double ratio){
        shooterVelocity[0] = vel;
        shooterVelocity[1] = ratio;
        setState(ShooterArmStates.VELOCITY);
    }

    public void setState(ShooterArmStates nextState){
        this.shooterArmState = nextState;
    }

    public boolean atShooterSetpoint(){
        if(Math.abs(shooterVelocity[0]/2.0 - inputs.shooterSpeedRPS[0]) < 0.5 && 
        Math.abs((shooterVelocity[0] * shooterVelocity[1])/2.0 - inputs.shooterSpeedRPS[1]) < 0.1){
            return true;
        }
        return false;
    }

    public double getCurrent(){
        return inputs.shooterCurrent[0];
    }

    public boolean atArmSetpoint(){
        if(Math.abs(armAngleDegrees - inputs.armPosDeg[0]) < 0.3){
            return true;
        }
        return false;
    
    }

    public double getArmDegrees(){
        return inputs.armPosDeg[0];
    }
    
}