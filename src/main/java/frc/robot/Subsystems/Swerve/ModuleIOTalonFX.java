package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.commons.Conversions;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.swerveConstants.moduleConstants;

/*
 * Changes
 * Updated Tune Values /
 * SteerFeedback is now FusedCanCoder instead of RotorSensor (remote, fused, synced) -> getting absolute data at all times /
 * Angle encoder has specified magnet offset where as perviously 0 /
 * Created VelocityTorqueControls to replace velocity controls /
 * Made all control requests synchronous (.UpdateFreqHz = 0) /
 * Added TorqueCurrentConfigs: PeakForwardTOrqueCurrent (-800-800), PeakReverseTorqueCurent(-800-800) =  slip current? / 
 * Latency compensate signal updating /
 * Optimized drive velocity depending on wheel turns /
 * Added getSignals()/
 * Added getCurrentSignals()/
 * Added getState()
 * Added setting brake/coast mode
 */
public class ModuleIOTalonFX{
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder angleEncoder;
    private double CANcoderOffset;
    private TalonFXConfiguration driveConfigs;
    private TalonFXConfiguration steerConfigs;
    private CANcoderConfiguration angleEncoderConfigs;


    private final StatusSignal<Angle> m_steerPosition;
    private final StatusSignal<Angle> m_drivePosition;
    private final StatusSignal<AngularVelocity> m_driveVelocity;
    private final StatusSignal<AngularVelocity> m_steerVelocity;
    private final StatusSignal<Current> driveStatorCurrent;
    private final StatusSignal<Current> driveSupplyCurrent;
    private final StatusSignal<Current> steerStatorCurrent;
    private final StatusSignal<Angle> absolutePositionRotations;

    private BaseStatusSignal signals[];
    private BaseStatusSignal currentSignals[];

    private PositionVoltage steerRequest;
    private VelocityVoltage velocityVoltageRequest;
    private VelocityTorqueCurrentFOC velocityTorqueRequest;
    private VoltageOut driveVoltageRequest;
    private VoltageOut steerVoltageRequest;

    private SwerveModulePosition m_internalState;

    public ModuleIOTalonFX(int driveID, int steerID, int CANcoderID, double CANcoderOffset, InvertedValue driveInvert, InvertedValue steerInvert, SensorDirectionValue CANcoderInvert) {
        driveMotor = new TalonFX(driveID, "canivore");
        steerMotor = new TalonFX(steerID, "canivore");
        angleEncoder = new CANcoder(CANcoderID, "canivore");
        this.CANcoderOffset = CANcoderOffset;
        m_internalState = new SwerveModulePosition();
        driveConfigs = new TalonFXConfiguration();
        steerConfigs = new TalonFXConfiguration();
        angleEncoderConfigs = new CANcoderConfiguration();

        steerRequest = new PositionVoltage(0).withEnableFOC(true);
        velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(true);
        velocityTorqueRequest = new VelocityTorqueCurrentFOC(0);
        driveVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        steerVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        steerRequest.UpdateFreqHz = 0;
        velocityVoltageRequest.UpdateFreqHz = 0;
        velocityTorqueRequest.UpdateFreqHz = 0;
        driveVoltageRequest.UpdateFreqHz = 0;
        steerVoltageRequest.UpdateFreqHz = 0;

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();

        var driveMotorOutputConfigs = driveConfigs.MotorOutput;
        driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        driveMotorOutputConfigs.Inverted = driveInvert;

        var driveFeedbackConfigs = driveConfigs.Feedback;
        driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveMotor.setPosition(0);

        var driveCurrentLimitConfigs = driveConfigs.CurrentLimits;
        driveCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        driveCurrentLimitConfigs.StatorCurrentLimit = swerveConstants.moduleConstants.driveStatorCurrentLimit;

        var driveTorqueCurrentConfigs = driveConfigs.TorqueCurrent;
        driveTorqueCurrentConfigs.PeakForwardTorqueCurrent = 100; //should be tuned to be slip current or smth
        driveTorqueCurrentConfigs.PeakReverseTorqueCurrent = -100;

        var driveOpenLoopConfigs = driveConfigs.OpenLoopRamps;
        driveOpenLoopConfigs.VoltageOpenLoopRampPeriod = swerveConstants.moduleConstants.rampRate;

        var driveSlot0Configs = driveConfigs.Slot0;
        
        driveSlot0Configs.kP = 0.14;
        driveSlot0Configs.kI = 0;
        driveSlot0Configs.kD = 0;
        driveSlot0Configs.kS = 0.115;
        driveSlot0Configs.kV = 0.133;
        driveSlot0Configs.kA = 1;

        var driveSlot1Configs = driveConfigs.Slot1;
        
        driveSlot1Configs.kP = 0.14;
        driveSlot1Configs.kI = 0;
        driveSlot1Configs.kD = 0;
        driveSlot1Configs.kS = 0.115;
        driveSlot1Configs.kV = 0;
        driveSlot1Configs.kA = 1;

        // STEER

        var steerMotorOutputConfigs = steerConfigs.MotorOutput;
        steerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        steerMotorOutputConfigs.Inverted = steerInvert;

        var steerFeedbackConfigs = steerConfigs.Feedback;
        steerFeedbackConfigs.FeedbackSensorSource =  FeedbackSensorSourceValue.RotorSensor;
        //steerFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        //steerFeedbackConfigs.SensorToMechanismRatio = 1.0;
        //steerFeedbackConfigs.RotorToSensorRatio = 150.0/7.0;


        var steerSlot0Configs = steerConfigs.Slot0;
        steerSlot0Configs.kP = 6;
        steerSlot0Configs.kI = 0;
        steerSlot0Configs.kD = 0.002;
        steerSlot0Configs.kS = 0.24;
        steerSlot0Configs.kV = 0.001;
        steerSlot0Configs.kA = 0.16;

        

        var steerCurrentLimitConfigs = steerConfigs.CurrentLimits;
        steerCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        steerCurrentLimitConfigs.StatorCurrentLimit = swerveConstants.moduleConstants.steerStatorCurrentLimit;

        // CANcoder
        var angleEncoderConfig = new CANcoderConfiguration();
        //angleEncoderConfig.MagnetSensor.MagnetOffset = CANcoderOffset;
        //angleEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        angleEncoderConfig.MagnetSensor.SensorDirection =  SensorDirectionValue.CounterClockwise_Positive;
           

        driveMotor.getConfigurator().apply(driveConfigs);
        steerMotor.getConfigurator().apply(steerConfigs);
        StatusCode stat = angleEncoder.getConfigurator().apply(angleEncoderConfigs);
        if(!stat.isOK()){
            System.out.println("Cancoder[" + CANcoderID + "] failed config: " + stat.toString());
        }

        m_steerPosition = steerMotor.getRotorPosition();
        m_drivePosition = driveMotor.getRotorPosition();
        m_driveVelocity = driveMotor.getRotorVelocity();
        m_steerVelocity = steerMotor.getRotorVelocity();
        driveStatorCurrent = driveMotor.getStatorCurrent();
        driveSupplyCurrent = driveMotor.getSupplyCurrent();
        steerStatorCurrent = steerMotor.getStatorCurrent();
        absolutePositionRotations = angleEncoder.getAbsolutePosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50, driveStatorCurrent, driveSupplyCurrent, steerStatorCurrent, absolutePositionRotations);

        signals = new BaseStatusSignal[4];
        signals[0] = m_steerPosition;
        signals[1] =  m_driveVelocity;
        signals[2] = m_steerVelocity;
        signals[3] = m_drivePosition;

        currentSignals = new BaseStatusSignal[4]; //current signals as in electrical current
        currentSignals[0] = driveStatorCurrent;
        currentSignals[1] = driveSupplyCurrent;
        currentSignals[2] = steerStatorCurrent;
        currentSignals[3] = absolutePositionRotations;
    }

    public SwerveModulePosition getPosition(boolean refresh) {
    if (refresh) {
      /* Refresh all signals */
      m_drivePosition.refresh();
      m_driveVelocity.refresh();
      m_steerPosition.refresh();
      m_steerVelocity.refresh();
    }

    /* Now latency-compensate our signals */
    double drive_rot =
        BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity).magnitude();
    double angle_rot =
        BaseStatusSignal.getLatencyCompensatedValue(m_steerPosition, m_steerVelocity).magnitude();
    
    angle_rot = angle_rot/swerveConstants.moduleConstants.steerGearRatio;

    m_internalState.distanceMeters = Conversions.RotationsToMeters(drive_rot, swerveConstants.moduleConstants.wheelCircumferenceMeters, swerveConstants.moduleConstants.driveGearRatio);

    /* Angle is already in terms of steer rotations */
    m_internalState.angle = Rotation2d.fromRotations(angle_rot);

    return m_internalState;
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(Conversions.RPStoMPS(m_driveVelocity.getValueAsDouble(), swerveConstants.moduleConstants.wheelCircumferenceMeters, swerveConstants.moduleConstants.driveGearRatio), m_internalState.angle);
  }


    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean TorqueFOC) {
        desiredState.optimize( m_internalState.angle);
        double steerMotorErrorRotations = desiredState.angle.getRotations() - m_internalState.angle.getRotations();
        double desiredVelocity = Conversions.MPStoRPS(desiredState.speedMetersPerSecond, moduleConstants.wheelCircumferenceMeters, moduleConstants.driveGearRatio);
        double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorErrorRotations));
        if (cosineScalar < 0.0) {
         cosineScalar = 0.0;
        }
        desiredVelocity *= cosineScalar;

        if(isOpenLoop){
            double driveVoltage = desiredVelocity;
            double angleDeg = desiredState.angle.getDegrees();

            setDriveVoltage(driveVoltage);
            setTurnAngle(angleDeg);
        }
        else if(!isOpenLoop){
            double driveVelocityRPS = desiredVelocity;
            double angleDeg = desiredState.angle.getDegrees();
            
            if(TorqueFOC){
                setDriveVelocityTorque(driveVelocityRPS);
            }
            else{
                setDriveVelocity(driveVelocityRPS);
            }
            setTurnAngle(angleDeg);
        }
    }

    public void setDriveVoltage(double volts) {
        driveMotor.setControl(driveVoltageRequest.withOutput(volts));
    }

    public void setDriveVelocityTorque(double velocityRPS){
        driveMotor.setControl(velocityTorqueRequest.withVelocity(velocityRPS).withSlot(1));
    }

    public void steerVoltage(double volts) {
        steerMotor.setControl(steerVoltageRequest.withOutput(volts));
    }

    public void setTurnAngle(double angleDeg) {
        steerMotor.setControl(steerRequest.withPosition(
                Conversions.DegreesToRotations(angleDeg, swerveConstants.moduleConstants.steerGearRatio)));
    }

    public void setDriveVelocity(double velocityRPS) {
        velocityVoltageRequest.Velocity = velocityRPS;
        driveMotor.setControl(velocityVoltageRequest);
    }

    public void resetToAbsolute() {
        double absolutePositionRotations = angleEncoder.getAbsolutePosition().getValueAsDouble() - CANcoderOffset;
        double absolutePositionSteerRotations = absolutePositionRotations * moduleConstants.steerGearRatio;
        steerMotor.setPosition(absolutePositionSteerRotations);
    }

    // Does this update?
    // steerPos, driveVel, steerVel, drivePos
    public BaseStatusSignal[] getSignals(){
        return signals;
    }
    
    // driveStator, driveSupply, steerStator
    public BaseStatusSignal[] getCurrentSignals(){
        return currentSignals;
    }

    public double getDriveVelocity(){
        return m_driveVelocity.getValueAsDouble();
    }


    public void setTurnBrakeMode(boolean brake){
        var steerMotorOutputConfigs = steerConfigs.MotorOutput;
        steerMotorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        steerMotor.getConfigurator().apply(steerConfigs);
        
    }

    public void setDriveBrakeMode(boolean brake){
        var driveMotorOutputConfigs = driveConfigs.MotorOutput;
        driveMotorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        driveMotor.getConfigurator().apply(driveConfigs);
        
    }

}