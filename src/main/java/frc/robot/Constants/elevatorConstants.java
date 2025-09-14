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
    public static final double CruiseVelocity = 150;
    public static final double Acceleration = 200;
    public static final double Jerk = 20000;

    /* Heights in Meters */
    public static final double L1 = 0;
    public static final double L2 = Units.inchesToMeters(6.5);
    public static final double L3 = Units.inchesToMeters(12.5);
    public static final double L4 = 0.655;//couldn't be bothered to convert back to inches lmao
}