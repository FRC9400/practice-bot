package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

import frc.commons.Conversions;

public class elevatorConstants {
    /* Inverts */
    public static final InvertedValue elevatorMotorInvert = InvertedValue.CounterClockwise_Positive;
    
    /* Mechanical Constants */
    public static final double gearRatio = 6;
    public static final double wheelCircumferenceMeters = Units.inchesToMeters(1.4397*Math.PI);
    
    /* Max and Min Heights */
    public static final double minHeightMeters = 0;
    public static final double maxHeightMeters = Units.inchesToMeters(23.925); //Without plate fix
    public static final double minHeightRotations = 0;
    public static final double maxHeightRotations = Conversions.metersToRotations(maxHeightMeters, wheelCircumferenceMeters, gearRatio);

    /* Current Limits */
    public static final double statorCurrentLimit = 70;

    /* PID Values*/
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;

    /* MotionMagic Values */
    public static final double CruiseVelocityUp = 0;
    public static final double AccelerationUp = 0;
    public static final double Jerk = 0;

    public static final double CruiseVelocityDown = 0;
    public static final double AccelerationDown = 0;
}
