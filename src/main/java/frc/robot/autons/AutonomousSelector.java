package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;



public class AutonomousSelector {
    private SendableChooser<modes> autonomousSelector = new SendableChooser<modes>();
    String mode;
        public enum modes{
            DO_NOTHING,
            PRELOAD_AMP,
            PRELOAD_MID,
            PRELOAD_SOURCE,
            FOUR_NOTE_MID,
            FOUR_NOTE_AMP,
            FOUR_NOTE_SOURCE,
            TWO_NOTE_MID,
            TWO_NOTE_AMP,
            TWO_NOTE_SOURCE

        };
    
    public AutonomousSelector(Swerve swerve, Superstructure superstructure){
        
        autonomousSelector.setDefaultOption(
            "Do Nothing", modes.DO_NOTHING
        );

        autonomousSelector.addOption("Preload Amp", modes.PRELOAD_AMP);

        autonomousSelector.addOption("Preload Mid", modes.PRELOAD_MID);

        autonomousSelector.addOption("Preload Source", modes.PRELOAD_SOURCE);

        autonomousSelector.addOption("Two Note Mid", modes.TWO_NOTE_MID);
        
        autonomousSelector.addOption("Two Note Amp", modes.TWO_NOTE_AMP);

        autonomousSelector.addOption("Two Note Source", modes.TWO_NOTE_SOURCE);

        autonomousSelector.addOption("Four Note Mid", modes.FOUR_NOTE_MID);

        autonomousSelector.addOption("Four Note Amp", modes.FOUR_NOTE_AMP);

        autonomousSelector.addOption("Four Note Source", modes.FOUR_NOTE_SOURCE);

        SmartDashboard.putData("Auto Choices", autonomousSelector);
    }

    public modes get(){
        return autonomousSelector.getSelected();
    }
}