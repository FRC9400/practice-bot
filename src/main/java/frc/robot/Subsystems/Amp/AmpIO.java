package frc.robot.Subsystems.Amp;

import org.littletonrobotics.junction.AutoLog;

public interface AmpIO {
    @AutoLog
    public static class AmpIOInputs{
        public double ampRollerAppliedVolts = 0.0;
        public double ampRollerTemp = 0.0;
        public double ampRollerCurrent = 0.0;
        public double ampRollerRPS = 0.0;
        public double ampRollerSetpointVotls = 0.0;
    }

    public default void updateInputs(AmpIOInputs inputs){}

    public default void setAmpRollerVoltage(double voltage) {}
}
