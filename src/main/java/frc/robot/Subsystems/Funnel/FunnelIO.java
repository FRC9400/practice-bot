package frc.robot.Subsystems.Funnel;

import org.littletonrobotics.junction.AutoLog;

public interface FunnelIO {
    @AutoLog
    public static class FunnelInputs{
        public double appliedVolts = 0;
        public double setpointVolts = 0;
        public double voltage = 0;
        public double velocityRPS = 0;
        public double currentAmps = 0;
        public double temperature = 0;
    }

    public default void updateInputs(FunnelInputs inputs){}
    public default void requestVoltage(double volts){}
}
