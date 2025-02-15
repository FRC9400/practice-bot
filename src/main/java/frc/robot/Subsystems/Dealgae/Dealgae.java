package frc.robot.Subsystems.Dealgae;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dealgae extends SubsystemBase {
    private final DealgaeIO dealgaeIO;
    private DealgaeIOInputsAutoLogged inputs = new DealgaeIOInputsAutoLogged();

    public Dealgae(DealgaeIO dealgaeIO){
        this.dealgaeIO = dealgaeIO;
    }

    @Override
    public void periodic(){
        dealgaeIO.updateInputs(inputs);
        Logger.processInputs("Dealgae", inputs);
    }

    public void requestVoltage(double voltage){
        dealgaeIO.requestVoltage(voltage);
    }
}
