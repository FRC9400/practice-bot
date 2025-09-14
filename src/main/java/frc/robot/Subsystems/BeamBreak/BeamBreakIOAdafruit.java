package frc.robot.Subsystems.BeamBreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIOAdafruit implements BeamBreakIO {
    private final DigitalInput beamBreak;
    private final boolean invert;

    public BeamBreakIOAdafruit(int id, boolean invert){
        this.beamBreak = new DigitalInput(id);
        this.invert = invert;
    }

    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.beamBroken = invert ? !beamBreak.get() : beamBreak.get();
      }
}