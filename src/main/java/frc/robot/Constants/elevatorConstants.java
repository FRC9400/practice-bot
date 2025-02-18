package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

import frc.commons.Conversions;

public class elevatorConstants {
    /* Inverts */
    public static final InvertedValue elevatorMotorInvert = InvertedValue.Clockwise_Positive;

    /* Neutral Modes */
    public static final NeutralModeValue elevatorNeutralMode = NeutralModeValue.Brake;
    
    /* Mechanical Constants */
    public static final double gearRatio = 6;
    public static final double wheelCircumferenceMeters = Units.inchesToMeters(1.4397*Math.PI);
    
    /* Max and Min Heights */
    public static final double minHeightMeters = 0;
    public static final double maxHeightMeters = Units.inchesToMeters(21); //Without plate fix
    public static final double minHeightRotations = 0;
    public static final double maxHeightRotations = Conversions.metersToRotations(maxHeightMeters, wheelCircumferenceMeters, gearRatio);

    /* Current Limits */
    public static final double statorCurrentLimit = 70;

    /* MotionMagic Values */
    public static final double CruiseVelocity = 40;
    public static final double Acceleration = 40;
    public static final double Jerk = 10000;
}
