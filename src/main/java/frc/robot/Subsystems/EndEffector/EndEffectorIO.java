package frc.robot.Subsystems.EndEffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    public static class EndEffectorInputs {
        public double appliedVolts = 0;
        public double setpointVolts = 0;
        public double voltage = 0;
        public double velocityRPS = 0;
        public double currentAmps = 0;
        public double tempFahrenheit = 0;
    }

    public default void updateInputs(EndEffectorInputs inputs){}

    public default void requestVoltage(double volts){}

}
