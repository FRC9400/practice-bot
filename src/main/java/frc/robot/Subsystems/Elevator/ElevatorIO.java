package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public double appliedVolts = 0;
        public double appliedMeters = 0;
        public double setpointVolts = 0;
        public double setpointMeters = 0;
        public double elevatorHeightMeters = 0;
        public double[] velocityMPS = new double [] {};
        public double[] voltage = new double [] {};
        public double[] velocityRPS = new double [] {};
        public double[] currentAmps = new double[] {};
        public double[] tempFahrenheit = new double[] {};    
    }

    public default void updateInputs(ElevatorIOInputs inputs){}

    public default void requestVoltage(double volts){}

    public default void requestMotionMagic(double meters){}

    public default void zeroSensor(double newValue){} 
    
    public default void resetMotionMagicConfigs(boolean down){}
}
