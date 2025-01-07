package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

import java.lang.Math;

public class shooterConstants {

    /* Motor Inverts */
    public static final InvertedValue leftShooterInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightShooterInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue leftArmInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue rightArmInvert = InvertedValue.CounterClockwise_Positive;

    /* Current Limits */
    public static final double shooterCurrentLimit = 85;
    public static final double armCurrentLimit = 60;

    public static double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI; 
    public static double shooterGearRatio = 0.5;
    public static double armGearRatio = 97.337962963;


}
