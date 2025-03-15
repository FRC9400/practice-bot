package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Swerve.Swerve;

public class AutonomousSelector {
    private SendableChooser<modes> autonomousSelector = new SendableChooser<modes>();
    String mode;
        public enum modes{
            DO_NOTHING,
            PRELOAD_DEALGAE_MID,
            PRELOAD_MID,
            PRELOAD_CAGE,
            LEAVE
        };
    
    public AutonomousSelector(Swerve swerve, Autos autos){
        autonomousSelector.setDefaultOption("Do Nothing", modes.DO_NOTHING);
        autonomousSelector.addOption("Preload and Dealgae from Mid", modes.PRELOAD_DEALGAE_MID);
        autonomousSelector.addOption("Preload from Mid", modes.PRELOAD_MID);
        autonomousSelector.addOption("Preload from Cage", modes.PRELOAD_CAGE);
        autonomousSelector.addOption("Leave", modes.LEAVE);
        SmartDashboard.putData("Auto Choices", autonomousSelector);
    }

    public modes get(){
        return autonomousSelector.getSelected();
    }
}
