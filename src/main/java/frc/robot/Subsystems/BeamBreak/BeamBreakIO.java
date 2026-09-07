package frc.robot.Subsystems.BeamBreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
    @AutoLog
    public static class BeamBreakIOInputs {
        public boolean beamBroken = false;
    }

    public default void updateInputs(BeamBreakIOInputs inputs) {}
    
}