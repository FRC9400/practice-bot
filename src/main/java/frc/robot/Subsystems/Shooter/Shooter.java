package frc.robot.Subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.util.Units;


public class Shooter {
    
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private ShooterStates shooterStates = ShooterStates.IDLE; //current state default to idle
    private double setpointVelocity;
    private double setpointVolts;//targets

    public Shooter(ShooterIO shooterIO){
        this.shooterIO = shooterIO;
    }

    public enum ShooterStates{
        IDLE, ZERO, VOLTAGE, VELOCITY, MM_VELOCITY
    }

    public void Loop(){
        shooterIO.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter", this.shooterStates);
        switch(shooterStates){
            case IDLE:
                shooterIO.requestVoltage(0);
                break;
            case ZERO:
                shooterIO.requestVelocity(0);
                break;
            case VOLTAGE:
                shooterIO.requestVoltage(setpointVolts);
                break;
            case VELOCITY:
                shooterIO.requestVelocity(setpointVelocity);
                break;
            case MM_VELOCITY:
                shooterIO.requestMMVelocity(setpointVelocity);
                break;
            default:
                break;
        } 
    }

    public void setState(ShooterStates nxt){
        this.shooterStates = nxt;
    }

    public boolean atSetPoint(){
        return Math.abs(inputs.shooterVelMPS[0]-setpointVelocity)<0.5; //check if wheel at right speed
    }

    public void requestIdle(){
        setState(ShooterStates.IDLE);
    }

    public void requestZero(){
        setState(ShooterStates.ZERO);
    }

    public void requestVoltage(double voltage){
        setpointVolts = voltage;
        setState(ShooterStates.VOLTAGE);
    }

    public void requestVelocity(double vel){
        setpointVelocity = vel;
        setState(ShooterStates.VELOCITY);
    }

    public void requestMMVelocity(double vel){
        setpointVelocity = vel;
        setState(ShooterStates.MM_VELOCITY);
    }

}
