package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class shooterConstants {
    /* Inverts */
    public static final InvertedValue shooterInvert = InvertedValue.CounterClockwise_Positive;

    /* Neutral Modes */
    public static final NeutralModeValue shooterNeutralMode = NeutralModeValue.Coast;

    /* Mechanical Constants */
    public static final double gearRatio = 1;
    public static final double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI;

    /* Current Limits */
    public static final double statorCurrentLimit = 70;
    public static final double supplyCurrentLimit = 40;
    public static final double supplyCurrentLowerLimit = 30;
    public static final double supplyCurrentLowerTime = 1.0;

    /* Slot 0 Gains */
    public static final double kP = 0.29;
    public static final double kI = 0;
    public static final double kD = 0.01;
    public static final double kS = 0.2;
    public static final double kV = 0.115;
    public static final double kA = 0;

    /* Motion Magic — meters per second per second */
    public static final double mmAcceleration = 15;
    public static final double mmJerk = 30;

    /* Setpoints — meters per second at the wheel surface */
    public static final double shootSpeedMPS = 12;
    public static final double toleranceMPS = 0.5;
}
