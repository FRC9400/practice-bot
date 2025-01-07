package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
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
    private final SensorDirectionValue CANcoderInvert;
    private TalonFXConfiguration driveConfigs;
    private TalonFXConfiguration steerConfigs;
    private CANcoderConfiguration angleEncoderConfigs;
    private CANcoderConfigurator angleEncoderConfigurator;
    private TalonFXConfigurator driveConfigurator;
    private TalonFXConfigurator steerConfigurator;

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

    LoggedTunableNumber drivekP = new LoggedTunableNumber("Drive/kP", 0);
    LoggedTunableNumber drivekD = new LoggedTunableNumber("Drive/kD", 0);
    LoggedTunableNumber drivekS = new LoggedTunableNumber("Drive/kS", 0);
    LoggedTunableNumber drivekV = new LoggedTunableNumber("Drive/kV", 0);

    LoggedTunableNumber steerkP = new LoggedTunableNumber("Steer/kP", 8);
    LoggedTunableNumber steerkD = new LoggedTunableNumber("Steer/kD", 0);
    LoggedTunableNumber steerkS = new LoggedTunableNumber("Steer/kS", 0);
    LoggedTunableNumber steerkV = new LoggedTunableNumber("Steer/kV", 0);

    LoggedTunableNumber voltage = new LoggedTunableNumber("Drive/Voltage", 0);

    public ModuleIOTalonFX(int driveID, int steerID, int CANcoderID, double CANcoderOffset, InvertedValue driveInvert,
            InvertedValue steerInvert, SensorDirectionValue CANcoderInvert) {
        driveMotor = new TalonFX(driveID, "canivore");
        steerMotor = new TalonFX(steerID, "canivore");
        angleEncoder = new CANcoder(CANcoderID, "canivore");
        this.CANcoderOffset = CANcoderOffset;
        this.driveInvert = driveInvert;
        this.steerInvert = steerInvert;
        this.CANcoderInvert = CANcoderInvert;
        driveConfigs = new TalonFXConfiguration();
        steerConfigs = new TalonFXConfiguration();
        angleEncoderConfigs = new CANcoderConfiguration();
        driveConfigurator = driveMotor.getConfigurator();
        steerConfigurator = steerMotor.getConfigurator();
        angleEncoderConfigurator = angleEncoder.getConfigurator();

        steerRequest = new PositionVoltage(0).withEnableFOC(true);
        velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(true);
        driveVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        steerVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();

        var driveMotorOutputConfigs = driveConfigs.MotorOutput;
        driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        driveMotorOutputConfigs.Inverted = driveInvert;
        driveMotorOutputConfigs.PeakForwardDutyCycle = 1.0;
        driveMotorOutputConfigs.PeakReverseDutyCycle = -1.0;

        var driveFeedbackConfigs = driveConfigs.Feedback;
        driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotor.setPosition(0);

        var driveCurrentLimitConfigs = driveConfigs.CurrentLimits;
        driveCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        driveCurrentLimitConfigs.StatorCurrentLimit = swerveConstants.moduleConstants.driveStatorCurrentLimit;

        var driveOpenLoopConfigs = driveConfigs.OpenLoopRamps;
        driveOpenLoopConfigs.VoltageOpenLoopRampPeriod = swerveConstants.moduleConstants.rampRate;

        var driveSlot0Configs = driveConfigs.Slot0;
        /* 
         //swan carpet
        driveSlot0Configs.kP = 0.13995; // 0.13995
        driveSlot0Configs.kI = 0.0;
        driveSlot0Configs.kD = 0.0;
        driveSlot0Configs.kS = 0.011412; //  0.011412
        driveSlot0Configs.kV = 0.12125;// 0.12125
        driveSlot0Configs.kA = 0.042716; // 0.042716*/

        //field carpet
        
        driveSlot0Configs.kP = 0.14922; //0.12006; // 0.13995
        driveSlot0Configs.kI = 0.0;
        driveSlot0Configs.kD = 0.0;
        driveSlot0Configs.kS = 0.18123; //0.21146; //  0.011412
        driveSlot0Configs.kV = 0.14061; //0.12209;// 0.12125
        driveSlot0Configs.kA = 0.0096859;// 0.013607; // 0.042716*/

        // STEER

        var steerMotorOutputConfigs = steerConfigs.MotorOutput;
        steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        steerMotorOutputConfigs.Inverted = steerInvert;

        var steerFeedbackConfigs = steerConfigs.Feedback;
        steerFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // steerFeedbackConfigs.FeedbackRemoteSensorID = CANcoderID;
        // steerFeedbackConfigs.RotorToSensorRatio =
        // swerveConstants.moduleConstants.steerGearRatio;
        // steerFeedbackConfigs.SensorToMechanismRatio = 1.0;
        // steerFeedbackConfigs.FeedbackRotorOffset = 0;

        var steerSlot0Configs = steerConfigs.Slot0;
        steerSlot0Configs.kP = 8.5142;
        steerSlot0Configs.kI = 0.0;
        steerSlot0Configs.kD = 0.069397;
        steerSlot0Configs.kS = 0.21052;
        steerSlot0Configs.kV = 0.1202;
        steerSlot0Configs.kA = 0.0010619;

        var steerCurrentLimitConfigs = steerConfigs.CurrentLimits;
        steerCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        steerCurrentLimitConfigs.StatorCurrentLimit = swerveConstants.moduleConstants.steerStatorCurrentLimit;

        // CANcoder
        var angleEncoderConfig = new CANcoderConfiguration();
        angleEncoderConfig.MagnetSensor.MagnetOffset = 0;
      //  angleEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        angleEncoderConfig.MagnetSensor.SensorDirection =  SensorDirectionValue.CounterClockwise_Positive;
           

        driveConfigurator.apply(driveConfigs);
        steerConfigurator.apply(steerConfigs);
        angleEncoderConfigurator.apply(angleEncoderConfigs);

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

    @Override
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

    @Override
    /* 
    public void updateTunableNumbers() {
        if (drivekD.hasChanged(drivekD.hashCode()) ||
                drivekS.hasChanged(drivekS.hashCode()) ||
                drivekP.hasChanged(drivekP.hashCode()) ||
                drivekV.hasChanged(drivekV.hashCode())) {
            var driveSlot0Configs = new Slot0Configs();
                    driveSlot0Configs.kP = 0.13995;
        driveSlot0Configs.kI = 0.0;
        driveSlot0Configs.kD = 0.0;
        driveSlot0Configs.kS = 0.011412;
        driveSlot0Configs.kV = 0.12125;
        driveSlot0Configs.kA = 0.042716;

            driveConfigurator.apply(driveSlot0Configs);
        }

        if (steerkD.hasChanged(steerkD.hashCode()) ||
                steerkS.hasChanged(steerkS.hashCode()) ||
                steerkP.hasChanged(steerkP.hashCode()) ||
                steerkV.hasChanged(steerkV.hashCode())) {
            var steerSlot0Configs = new Slot0Configs();
            steerSlot0Configs.kP = 11.136;
            steerSlot0Configs.kI = 0.0;
            steerSlot0Configs.kD = 0.13881;
            steerSlot0Configs.kS = 0.32456;
            steerSlot0Configs.kV = 0.12174;
            steerSlot0Configs.kA = 0.0019929;

            steerConfigurator.apply(steerSlot0Configs);
        }
    }*/

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