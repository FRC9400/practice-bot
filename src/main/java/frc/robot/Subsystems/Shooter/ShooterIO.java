package frc.robot.Subsystems.Shooter;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs{
        public double appliedVolts = 0.0; //voltage reaching motor rn
        public double shooterSetpointMPS = 0.0; //target in meters per second
        public double velocitySetpointRPS; //target vel
        public double[] shooterVelMPS = new double[] {0.0}; //surface speed of each wheel
        public double[] shooterVelRPS = new double[] {0.0}; //same thing but rps
        public double[] currentAmps = new double[] {0.0}; //how hard each motor working 
        public double[] shooterVoltage = new double[] {0.0}; //same thing but per motor
        public double[] temp = new double[] {0.0}; //motor temp
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    public default void requestVelocity(double velocity){
        //closed loop - spin at _ speed
    }
    public default void requestMMVelocity(double velocityMPS){}
    public default void zeroVelocity(){}
    public default void requestVoltage(double volts){
        //open loop, put _ volts across motor
    } 

}
