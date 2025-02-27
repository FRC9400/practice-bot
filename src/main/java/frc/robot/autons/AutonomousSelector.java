package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Swerve.Swerve;

public class AutonomousSelector {
    private SendableChooser<modes> autonomousSelector = new SendableChooser<modes>();
    String mode;
        public enum modes{
            DO_NOTHING,
            TUNE_X,
            TUNE_Y,
            TUNE_THETA,
            POS1_PRELOAD_TO_I,
            POS2_PRELOAD_TO_I,
            POS3_PRELOAD_TO_F,
            POS1_TWOPIECE_I_J,
            POS2_TWOPIECE_I_J,
            POS3_TWOPIECE_F_E
        };
    
    public AutonomousSelector(Swerve swerve, Autos autos){
        autonomousSelector.setDefaultOption("Do Nothing", modes.DO_NOTHING);
        autonomousSelector.addOption("Tune X", modes.TUNE_X);
        autonomousSelector.addOption("Tune Y", modes.TUNE_Y);
        autonomousSelector.addOption("Tune Theta", modes.TUNE_THETA);
        autonomousSelector.addOption("Position 1 Preload to F", modes.POS1_PRELOAD_TO_I);
        autonomousSelector.addOption("Position 2 Preload to I", modes.POS2_PRELOAD_TO_I);
        autonomousSelector.addOption("Position 3 Preload to F", modes.POS3_PRELOAD_TO_F);
        autonomousSelector.addOption("Position 1 Two Piece to I then J", modes.POS1_TWOPIECE_I_J);
        autonomousSelector.addOption("Position 2 Two Piece to I then J", modes.POS2_TWOPIECE_I_J);
        autonomousSelector.addOption("Position 3 Two Piece to F then E", modes.POS3_TWOPIECE_F_E);
        SmartDashboard.putData("Auto Choices", autonomousSelector);
    }

    public modes get(){
        return autonomousSelector.getSelected();
    }
}
