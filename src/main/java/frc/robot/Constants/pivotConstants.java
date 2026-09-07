package frc.robot.Constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class pivotConstants {
    /* Inverts */
    public static final InvertedValue pivotInvert = InvertedValue.Clockwise_Positive;

    /* Neutral Modes */
    public static final NeutralModeValue pivotNeutralMode = NeutralModeValue.Brake;

    /* Mechanical Constants */
    public static final double gearRatio = 60;

    /* Soft Limits (degrees, 0 = horizontal) */
    public static final double minAngleDegrees = 0;
    public static final double maxAngleDegrees = 60;

    /* Current Limits */
    public static final double statorCurrentLimit = 40;

    /* Motion Magic — rotor rotations per second, /s^2, /s^3 */
    public static final double cruiseVelocity = 40;
    public static final double acceleration = 80;
    public static final double jerk = 800;

    /* Slot 0 Gains */
    public static final double kP = 2.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.1;
    public static final double kV = 0;
    public static final double kG = 0.35;
    public static final GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;

    /* Angles (degrees) */
    public static final double stowAngleDegrees = 5;
    public static final double subwooferAngleDegrees = 55;
    public static final double podiumAngleDegrees = 30;
    public static final double toleranceDegrees = 1.0;
}