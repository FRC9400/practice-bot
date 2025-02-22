package frc.robot.autons;

import edu.wpi.first.math.controller.PIDController;

public class AutoConstants {
    public static PIDController xController = new PIDController(0.77, 0.0, 0.01);
    public static PIDController yController = new PIDController(0.2, 0.0, 0.0);
    public static PIDController headingController = new PIDController(13.25, 0.0, 0.09);
}