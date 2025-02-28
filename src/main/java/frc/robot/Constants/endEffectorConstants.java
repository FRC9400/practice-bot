package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class endEffectorConstants {
    /* Inverts */
    public static final InvertedValue leftEndEffectorMotorInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightEndEffectorMotorInvert = InvertedValue.CounterClockwise_Positive;

    /* Current Limits */
    public static final double statorCurrentLimit = 50;
}