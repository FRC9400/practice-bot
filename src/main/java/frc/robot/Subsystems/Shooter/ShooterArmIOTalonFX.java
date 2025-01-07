package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.shooterConstants;

public class ShooterArmIOTalonFX implements ShooterArmIO{
    private final TalonFX leftShooter = new TalonFX(canIDConstants.leftShooterMotor, "rio");
    private final TalonFX rightShooter = new TalonFX(canIDConstants.rightShooterMotor, "rio");
    private final TalonFX leftArm = new TalonFX(canIDConstants.leftArmMotor, "rio");
    private final TalonFX rightArm = new TalonFX(canIDConstants.rightArmMotor, "rio");
    
    private TalonFXConfiguration leftShooterConfigs;
    private TalonFXConfiguration rightShooterConfigs;
    private TalonFXConfiguration leftArmConfigs;

    private final StatusSignal<Current> leftShooterCurrent = leftShooter.getStatorCurrent();
    private final StatusSignal<Current> rightShooterCurrent = rightShooter.getStatorCurrent();
    private final StatusSignal<Temperature> leftShooterTemp = leftShooter.getDeviceTemp();
    private final StatusSignal<Temperature> rightShooterTemp = rightShooter.getDeviceTemp();
    private final StatusSignal<AngularVelocity> leftShooterSpeedRPS = leftShooter.getRotorVelocity();
    private final StatusSignal<AngularVelocity> rightShooterSpeedRPS = rightShooter.getRotorVelocity();

    private final StatusSignal<Current> leftArmCurrent = leftArm.getStatorCurrent();
    private final StatusSignal<Current> rightArmCurrent = rightArm.getStatorCurrent();
    private final StatusSignal<Temperature> leftArmTemp = leftArm.getDeviceTemp();
    private final StatusSignal<Temperature> rightArmTemp = rightArm.getDeviceTemp();
    private final StatusSignal<Angle> leftArmPos = leftArm.getRotorPosition();
    private final StatusSignal<Angle> rightArmPos = rightArm.getRotorPosition();
    private final StatusSignal<AngularVelocity> leftArmRPS = leftArm.getRotorVelocity();
    private final StatusSignal<AngularVelocity> rightArmRPS = rightArm.getRotorVelocity();
    
    private double leftShooterSetpointRPS = 0;
    private double rightShooterSetpointRPS = 0;
    private double leftArmSetpointDegrees = 0;

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private VelocityVoltage rightShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);

    private VoltageOut leftArmRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private MotionMagicVoltage leftArmMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
    
    private PositionVoltage leftArmPositionRequest = new PositionVoltage(0).withEnableFOC(true);

    public ShooterArmIOTalonFX() {
        leftShooterConfigs = new TalonFXConfiguration();
        rightShooterConfigs = new TalonFXConfiguration();
        leftArmConfigs = new TalonFXConfiguration();

        var leftShooterMotorConfigs = leftShooterConfigs.MotorOutput;
        var rightShooterMotorConfigs = rightShooterConfigs.MotorOutput;
    
        leftShooterMotorConfigs.Inverted = shooterConstants.leftShooterInvert;
        leftShooterMotorConfigs.NeutralMode = NeutralModeValue.Coast;

        rightShooterMotorConfigs.Inverted = shooterConstants.rightShooterInvert;
        rightShooterMotorConfigs.NeutralMode = NeutralModeValue.Coast;

        var leftShooterCurrentConfigs = leftShooterConfigs.CurrentLimits;
        leftShooterCurrentConfigs.StatorCurrentLimit = shooterConstants.shooterCurrentLimit;
        leftShooterCurrentConfigs.StatorCurrentLimitEnable = true;


        var rightShooterCurrentConfigs = rightShooterConfigs.CurrentLimits;
        rightShooterCurrentConfigs.StatorCurrentLimit = shooterConstants.shooterCurrentLimit;
        rightShooterCurrentConfigs.StatorCurrentLimitEnable = true;

        var leftShooterSlot0Configs = leftShooterConfigs.Slot0;
        leftShooterSlot0Configs.kP = 0.775;
        leftShooterSlot0Configs.kI = 0.0;
        leftShooterSlot0Configs.kD = 0.0;
        leftShooterSlot0Configs.kS = 0.4;
        leftShooterSlot0Configs.kV = 0.153;
        leftShooterSlot0Configs.kA = 0.0;

        var rightShooterSlot0Configs = rightShooterConfigs.Slot0;
        rightShooterSlot0Configs.kP = 0.775;
        rightShooterSlot0Configs.kI = 0.0;
        rightShooterSlot0Configs.kD = 0.0;
        rightShooterSlot0Configs.kS = 0.4;
        rightShooterSlot0Configs.kV = 0.153;
        rightShooterSlot0Configs.kA = 0.0;

        var leftArmMotorConfigs = leftArmConfigs.MotorOutput;

        leftArmMotorConfigs.Inverted = shooterConstants.leftArmInvert;
        leftArmMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        var leftArmCurrentConfigs = leftArmConfigs.CurrentLimits;

        leftArmCurrentConfigs.StatorCurrentLimit = shooterConstants.armCurrentLimit;
        leftArmCurrentConfigs.StatorCurrentLimitEnable = true;

        var leftArmSlot0Configs = leftArmConfigs.Slot0;
        leftArmSlot0Configs.kP = 10;
        leftArmSlot0Configs.kI = 0.0;
        leftArmSlot0Configs.kD = 0.0;
        leftArmSlot0Configs.kS = 0.23;
        leftArmSlot0Configs.kV = 0.25;
        leftArmSlot0Configs.kA = 0.010154;
        leftArmSlot0Configs.kG = 0.27;
        leftArmSlot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var leftArmSlot1Configs = leftArmConfigs.Slot1;
        leftArmSlot1Configs.kP = 0.0;
        leftArmSlot1Configs.kI = 0.0;
        leftArmSlot1Configs.kD = 0.0;
        leftArmSlot1Configs.kS = 0.0;
        leftArmSlot1Configs.kV = 0.0;
        leftArmSlot1Configs.kA = 0.0;
        leftArmSlot1Configs.kG = 0.0;
        leftArmSlot1Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = leftArmConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 75;
        motionMagicConfigs.MotionMagicAcceleration = 150;
        motionMagicConfigs.MotionMagicJerk = 10000;

        var feedbackConfigs = leftArmConfigs.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        leftShooter.getConfigurator().apply(leftShooterConfigs);
        rightShooter.getConfigurator().apply(rightShooterConfigs);
        rightArm.setControl(new Follower(leftArm.getDeviceID(), true));
        leftArm.getConfigurator().apply(leftArmConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
           leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS,
            leftArmCurrent,
            rightArmCurrent,
            leftArmTemp,
            rightArmTemp,
            leftArmPos,
            rightArmPos,
            leftArmRPS,
            rightArmRPS
            );

        leftShooter.optimizeBusUtilization();
        rightShooter.optimizeBusUtilization();
    }

    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftShooterCurrent,
            rightShooterCurrent,
            leftShooterTemp,
            rightShooterTemp,
            leftShooterSpeedRPS,
            rightShooterSpeedRPS,
            leftArmCurrent,
            rightArmCurrent,
            leftArmTemp,
            rightArmTemp,
            leftArmPos,
            rightArmPos,
            leftArmRPS,
            rightArmRPS
        );

        inputs.shooterAppliedVolts = shootRequestVoltage.Output;
        inputs.shooterCurrent = new double[] {leftShooterCurrent.getValueAsDouble(),
                rightShooterCurrent.getValueAsDouble() };
        inputs.shooterTemp = new double[] {leftShooterTemp.getValueAsDouble(),
                rightShooterTemp.getValueAsDouble() };
        inputs.shooterSpeedRPS = new double[] {leftShooterSpeedRPS.getValueAsDouble(),
                rightShooterSpeedRPS.getValueAsDouble() };
        inputs.shooterSetpointsRPS = new double[] {leftShooterSetpointRPS, rightShooterSetpointRPS};
    
        inputs.armAppliedVolts = leftArmRequestVoltage.Output;
        inputs.armSetpointDeg = leftArmSetpointDegrees;
        inputs.armSetpointRot = Conversions.DegreesToRotations(leftArmSetpointDegrees, shooterConstants.armGearRatio);
        inputs.armPosRot = new double[] {leftArmPos.getValueAsDouble(), rightArmPos.getValueAsDouble()};
        inputs.armPosDeg = new double[] {Conversions.RotationsToDegrees(leftArmPos.getValueAsDouble(), shooterConstants.armGearRatio), Conversions.RotationsToDegrees(rightArmPos.getValueAsDouble(), shooterConstants.armGearRatio)};

        inputs.armCurrent = new double[] {leftArmCurrent.getValueAsDouble(), rightArmCurrent.getValueAsDouble()};
        inputs.armTemp = new double[] {leftArmTemp.getValueAsDouble(), rightArmTemp.getValueAsDouble()};
        inputs.armRPS = new double[] {leftArmRPS.getValueAsDouble(), rightArmRPS.getValueAsDouble()};
    }


    public void requestShooterVoltage(double voltage) {
        leftShooter.setControl(shootRequestVoltage.withOutput(voltage));
        rightShooter.setControl(shootRequestVoltage.withOutput(voltage));

        //rightShooter.setControl(new Follower(leftShooter.getDeviceID(), true));
    }

    public void requestVelocity(double velocity, double ratio){
        leftShooterSetpointRPS = velocity/2;
        rightShooterSetpointRPS = leftShooterSetpointRPS * ratio;
        leftShooter.setControl(leftShootRequestVelocity.withVelocity(leftShooterSetpointRPS));
        rightShooter.setControl(rightShootRequestVelocity.withVelocity(rightShooterSetpointRPS));
    }

    public void requestArmVoltage(double voltage) {
        leftArm.setControl(leftArmRequestVoltage.withOutput(voltage));
    }

    public void requestMotionMagicSetpoint(double angleDegrees) {
        leftArmSetpointDegrees = angleDegrees;
        double leftArmSetpointRotations = Conversions.DegreesToRotations(angleDegrees, shooterConstants.armGearRatio);
        leftArm.setControl(leftArmMotionMagicRequest.withPosition(leftArmSetpointRotations).withSlot(0));
    }

    public void requestPositionSetpoint(double angleDegrees){
        leftArmSetpointDegrees = angleDegrees;
        double leftArmSetpointRotations = Conversions.DegreesToRotations(angleDegrees, shooterConstants.armGearRatio);
        leftArm.setControl(leftArmPositionRequest.withPosition(leftArmSetpointRotations).withSlot(1));
    }
}
