package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public double pivotVoltage = 0;
        public double pivotSetpointVolts = 0;
        public double pivotAppliedVolts = 0;
        public double pivotCurrentAmps = 0;
        public double pivotSetpointDeg = 0;
        public double pivotSetpointRot = 0; 
        public double pivotPositionDeg = 0;
        public double pivotPositionRot = 0;
        public double pivotTempFahrenheit = 0;
        public double pivotVelocityRPS = 0;

        public double rollerAppliedVolts = 0;
        public double rollerSetpointVolts = 0;
        public double rollerVoltage = 0;
        public double rollerCurrentAmps = 0;
        public double rollerVelocityRPS = 0;
        public double rollerTempFahrenheit = 0;
    }

    public default void updateInputs(IntakeIOInputs inputs){}

    public default void requestPivotVoltage(double volts){}

    public default void requestMotionMagic(double degrees){}

    public default void requestRollerVoltage(double volts){}

    public default void pivotConfiguration(){}

    public default void updateTunableNumbers() {}

    public default void zeroSensor(){}

}
