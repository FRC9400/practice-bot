package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import frc.commons.LoggedTunableNumber;
import frc.commons.Conversions;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.swerveConstants.moduleConstants;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder angleEncoder;
    private final double CANcoderOffset;
    private final InvertedValue driveInvert;
    private final InvertedValue steerInvert;
    private TalonFXConfiguration driveConfigs;
    private TalonFXConfiguration steerConfigs;
    private CANcoderConfiguration angleEncoderConfigs;

    private final StatusSignal<Angle> steerPos;
    private final StatusSignal<Angle> drivePos;
    private final StatusSignal<AngularVelocity> driveVelRPS;
    private final StatusSignal<Temperature> driveTemp;
    private final StatusSignal<Temperature> steerTemp;
    private final StatusSignal<Current> driveAmps;
    private final StatusSignal<Current> steerAmps;
    private final StatusSignal<Angle> absolutePositionRotations;

    private PositionVoltage steerRequest;
    private VelocityVoltage velocityVoltageRequest;
    private VoltageOut driveVoltageRequest;
    private VoltageOut steerVoltageRequest;

    LoggedTunableNumber drivekP = new LoggedTunableNumber("Drive/kP", 0.08);
    LoggedTunableNumber drivekI = new LoggedTunableNumber("Drive/kI", 0);
    LoggedTunableNumber drivekD = new LoggedTunableNumber("Drive/kD", 0);
    LoggedTunableNumber drivekS = new LoggedTunableNumber("Drive/kS", 0.13);
    LoggedTunableNumber drivekV = new LoggedTunableNumber("Drive/kV", 0.14);
    LoggedTunableNumber drivekA = new LoggedTunableNumber("Drive/kA",  0.15);

    LoggedTunableNumber steerkP = new LoggedTunableNumber("Steer/kP", 3);
    LoggedTunableNumber steerkI = new LoggedTunableNumber("Steer/kI", 0);
    LoggedTunableNumber steerkD = new LoggedTunableNumber("Steer/kD", 0);
    LoggedTunableNumber steerkS = new LoggedTunableNumber("Steer/kS", 0.26814);
    LoggedTunableNumber steerkV = new LoggedTunableNumber("Steer/kV", 0.000966);
    LoggedTunableNumber steerkA = new LoggedTunableNumber("Steer/kA", 0);

    public ModuleIOTalonFX(int driveID, int steerID, int CANcoderID, double CANcoderOffset, InvertedValue driveInvert, InvertedValue steerInvert, SensorDirectionValue CANcoderInvert) {
        driveMotor = new TalonFX(driveID, "canivore");
        steerMotor = new TalonFX(steerID, "canivore");
        angleEncoder = new CANcoder(CANcoderID, "canivore");
        this.driveInvert = driveInvert;
        this.steerInvert = steerInvert;
        this.CANcoderOffset = CANcoderOffset;
        driveConfigs = new TalonFXConfiguration();
        steerConfigs = new TalonFXConfiguration();
        angleEncoderConfigs = new CANcoderConfiguration();

        steerRequest = new PositionVoltage(0).withEnableFOC(true);
        velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(true);
        driveVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        steerVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        
        // CANcoder
        var angleEncoderConfig = new CANcoderConfiguration();
        angleEncoderConfig.MagnetSensor.MagnetOffset = 0;
      //  angleEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        angleEncoderConfig.MagnetSensor.SensorDirection =  SensorDirectionValue.CounterClockwise_Positive;
           
        driveConfig();

        angleEncoder.getConfigurator().apply(angleEncoderConfigs);

        steerPos = steerMotor.getRotorPosition();
        drivePos = driveMotor.getRotorPosition();
        driveVelRPS = driveMotor.getRotorVelocity();
        driveTemp = driveMotor.getDeviceTemp();
        steerTemp = steerMotor.getDeviceTemp();
        driveAmps = driveMotor.getStatorCurrent();
        steerAmps = steerMotor.getStatorCurrent();
        absolutePositionRotations = angleEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                steerPos,
                drivePos,
                driveVelRPS,
                driveTemp,
                steerTemp,
                driveAmps,
                steerAmps);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
    }

    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                steerPos,
                drivePos,
                driveVelRPS,
                driveTemp,
                steerTemp,
                driveAmps,
                steerAmps);

        inputs.driveVelocityMetersPerSec = Conversions.RPStoMPS(driveVelRPS.getValueAsDouble(),
                swerveConstants.moduleConstants.wheelCircumferenceMeters,
                swerveConstants.moduleConstants.driveGearRatio);
        inputs.driveAppliedVolts = driveVoltageRequest.Output;
        inputs.driveCurrentAmps = driveAmps.getValueAsDouble();
        inputs.driveTempCelcius = driveTemp.getValueAsDouble();
        inputs.driveDistanceMeters = Conversions.RotationsToMeters(drivePos.getValueAsDouble(),
                swerveConstants.moduleConstants.wheelCircumferenceMeters,
                swerveConstants.moduleConstants.driveGearRatio);
        inputs.driveOutputPercent = driveMotor.get();
        inputs.rawDriveRPS = driveVelRPS.getValueAsDouble();

        inputs.moduleAngleRads = Units.degreesToRadians(
                Conversions.RotationsToDegrees(steerPos.getValueAsDouble(), swerveConstants.moduleConstants.steerGearRatio));
        inputs.moduleAngleDegs = Conversions.RotationsToDegrees(steerPos.getValueAsDouble(),
                swerveConstants.moduleConstants.steerGearRatio);
        inputs.rawAbsolutePositionRotations = absolutePositionRotations.getValueAsDouble();
        inputs.absolutePositionRadians = absolutePositionRotations.getValueAsDouble() * 2 * Math.PI;
        inputs.absolutePositionDegrees = absolutePositionRotations.getValueAsDouble() * 360;
        inputs.turnAppliedVolts = steerVoltageRequest.Output;
        inputs.turnCurrentAmps = steerAmps.getValueAsDouble();
        inputs.turnTempCelcius = steerTemp.getValueAsDouble();
    }

    public void updateTunableNumbers(){
        if (
            drivekD.hasChanged(drivekD.hashCode()) ||
            drivekS.hasChanged(drivekS.hashCode()) ||
            drivekA.hasChanged(drivekA.hashCode()) ||
            drivekV.hasChanged(drivekV.hashCode()) ||
            drivekP.hasChanged(drivekP.hashCode()) ||
            drivekI.hasChanged(drivekI.hashCode()) ||
            steerkD.hasChanged(steerkD.hashCode()) ||
            steerkS.hasChanged(steerkS.hashCode()) ||
            steerkA.hasChanged(steerkA.hashCode()) ||
            steerkV.hasChanged(steerkV.hashCode()) ||
            steerkP.hasChanged(steerkP.hashCode()) ||
            steerkI.hasChanged(steerkI.hashCode())
        ) {
            driveConfig();
        }
    }

    public void driveConfig(){
        var driveMotorOutputConfigs = driveConfigs.MotorOutput;
        driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        driveMotorOutputConfigs.Inverted = driveInvert;

        var driveFeedbackConfigs = driveConfigs.Feedback;
        driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotor.setPosition(0);

        var driveCurrentLimitConfigs = driveConfigs.CurrentLimits;
        driveCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        driveCurrentLimitConfigs.StatorCurrentLimit = swerveConstants.moduleConstants.driveStatorCurrentLimit;

        var driveOpenLoopConfigs = driveConfigs.OpenLoopRamps;
        driveOpenLoopConfigs.VoltageOpenLoopRampPeriod = swerveConstants.moduleConstants.rampRate;

        var driveSlot0Configs = driveConfigs.Slot0;
        
        driveSlot0Configs.kP = drivekP.get();
        driveSlot0Configs.kI = drivekI.get();
        driveSlot0Configs.kD = drivekD.get();
        driveSlot0Configs.kS = drivekS.get();
        driveSlot0Configs.kV = drivekV.get();
        driveSlot0Configs.kA = drivekA.get();

        var steerMotorOutputConfigs = steerConfigs.MotorOutput;
        steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        steerMotorOutputConfigs.Inverted = steerInvert;

        var steerFeedbackConfigs = steerConfigs.Feedback;
        steerFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        var steerSlot0Configs = steerConfigs.Slot0;
        steerSlot0Configs.kP = steerkP.get();
        steerSlot0Configs.kI = steerkI.get();
        steerSlot0Configs.kD = steerkD.get();
        steerSlot0Configs.kS = steerkS.get();
        steerSlot0Configs.kV = steerkV.get();
        steerSlot0Configs.kA = steerkA.get();

        var steerCurrentLimitConfigs = steerConfigs.CurrentLimits;
        steerCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        steerCurrentLimitConfigs.StatorCurrentLimit = swerveConstants.moduleConstants.steerStatorCurrentLimit;

        driveMotor.getConfigurator().apply(driveConfigs);
        steerMotor.getConfigurator().apply(steerConfigs);
    }

    public void setDesiredState(SwerveModuleState optimizedDesiredStates, boolean isOpenLoop) {
        if(isOpenLoop){
            double driveVoltage = optimizedDesiredStates.speedMetersPerSecond * 7 ;
            double angleDeg = optimizedDesiredStates.angle.getDegrees();

            setDriveVoltage(driveVoltage);
            setTurnAngle(angleDeg);
        }
        else if(!isOpenLoop){
            double driveVelocity = optimizedDesiredStates.speedMetersPerSecond;
            double angleDeg = optimizedDesiredStates.angle.getDegrees();

            setDriveVelocity(driveVelocity, true);
            setTurnAngle(angleDeg);
        }
    }

    public void setDriveVoltage(double volts) {
        driveMotor.setControl(driveVoltageRequest.withOutput(volts));
    }

    public void steerVoltage(double volts) {
        steerMotor.setControl(steerVoltageRequest.withOutput(volts));
    }

    public void setTurnAngle(double angleDeg) {
        steerMotor.setControl(steerRequest.withPosition(
                Conversions.DegreesToRotations(angleDeg, swerveConstants.moduleConstants.steerGearRatio)));
    }

    public void resetToAbsolute() {
        double absolutePositionRotations = angleEncoder.getAbsolutePosition().getValueAsDouble() - CANcoderOffset;
        double absolutePositionSteerRotations = absolutePositionRotations * moduleConstants.steerGearRatio;
        steerMotor.setPosition(absolutePositionSteerRotations);
    }

    public void setDriveVelocity(double velocityMetersPerSecond, boolean auto) {
        velocityVoltageRequest.Velocity = Conversions.MPStoRPS(velocityMetersPerSecond,
                swerveConstants.moduleConstants.wheelCircumferenceMeters,
                swerveConstants.moduleConstants.driveGearRatio);
        driveMotor.setControl(velocityVoltageRequest);
    }

}