package frc.robot.Subsystems.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs{
        public double appliedVolts = 0;
        public double setpointVolts = 0;
        public double setpointDegrees = 0;
        public double pivotAngleDegrees = 0;
        public double velocityRPS = 0;
        public double currentAmps = 0;
        public double tempFahrenheit = 0;
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void requestVoltage(double volts) {}

    public default void requestMotionMagic(double degrees) {}

    public default void zeroSensor(double newValue) {}
}