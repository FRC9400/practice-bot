package frc.robot.Subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorInputs {
        public double[] appliedVolts = new double [] {};
        public double[] setpointVolts = new double [] {};
        public double[] velocityRPS = new double [] {};
        public double[] currentAmps = new double[] {};
        public double[] tempFahrenheit = new double[] {};  
    }

    public default void updateInputs(EndEffectorInputs inputs){}

    public default void requestVoltage(double volts, double ratio){}

}
