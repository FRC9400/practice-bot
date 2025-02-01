package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Subsystems.Swerve.Swerve;

public class AutonomousSelector {
    private SendableChooser<modes> autonomousSelector = new SendableChooser<modes>();
    String mode;
        public enum modes{
            DO_NOTHING,
            RESET_POSE,
            TEST_CHOREO
        };
    
    public AutonomousSelector(Swerve swerve, Autos autos){
        autonomousSelector.setDefaultOption("Do Nothing", modes.DO_NOTHING);
        autonomousSelector.addOption("Reset Pose", modes.RESET_POSE);
        autonomousSelector.addOption("Test Choreo", modes.TEST_CHOREO);
    }

    public modes get(){
        return autonomousSelector.getSelected();
    }
}
