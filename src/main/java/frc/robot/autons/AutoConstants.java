package frc.robot.autons;

import edu.wpi.first.math.controller.PIDController;

public class AutoConstants {
    public static PIDController xController = new PIDController(3.0, 0.0, 0.0);
    public static PIDController yController = new PIDController(3.0, 0.0, 0.0);
    public static PIDController headingController = new PIDController(1.25, 0.0, 0.0);
}