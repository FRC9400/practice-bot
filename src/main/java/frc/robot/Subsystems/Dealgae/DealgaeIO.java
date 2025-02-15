package frc.robot.Subsystems.Dealgae;

import org.littletonrobotics.junction.AutoLog;

public interface DealgaeIO {
    @AutoLog
    public static class DealgaeIOInputs {
        public double appliedVolts = 0;
        public double setpointVolts = 0;
        public double velocityRPS = 0;
        public double currentAmps = 0;
        public double tempFahrenheit = 0;
    }

    public default void updateInputs(DealgaeIOInputs inputs){}

    public default void requestVoltage(double volts){}
}