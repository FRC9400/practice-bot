package frc.robot.Subsystems.EndEffector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {
    private final EndEffectorIO endEffectorIO;
    private EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();
   
    public EndEffector(EndEffectorIO endEffectorIO){
        this.endEffectorIO = endEffectorIO;
    }

    @Override
    public void periodic(){
        endEffectorIO.updateInputs(inputs);
        Logger.processInputs("End Effector", inputs);
    }
}
