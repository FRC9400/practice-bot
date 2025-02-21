package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

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
    public static final double maxHeightMeters = Units.inchesToMeters(24);
    
    /* Current Limits */
    public static final double statorCurrentLimit = 80;

    /* MotionMagic Values */
    public static final double CruiseVelocity = 60;
    public static final double Acceleration = 80;
    public static final double Jerk = 10000;

    /* Heights in Meters */
    public static final double L1 = 0;
    public static final double L2 = Units.inchesToMeters(5);
    public static final double L3 = Units.inchesToMeters(12.467784);
    public static final double L3Algae = Units.inchesToMeters(19.315);
    public static final double L4 = Units.inchesToMeters(23.925);
}
