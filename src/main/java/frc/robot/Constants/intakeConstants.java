package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class intakeConstants {
    /* Inverts */
    public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue rollerInvert = InvertedValue.CounterClockwise_Positive;

    /* Current Limits */
    public static final double pivotCurrentLimit = 70;
    public static final double rollerCurrentLimit = 50;

    /* Neutral Mode */
    public static final NeutralModeValue pivotNeutralMode = NeutralModeValue.Brake;

    /* Gear Ratios */
    public static final double pivotGearRatio = 33.523;

}