package frc.robot.Subsystems.Pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.pivotConstants;


public class PivotIOTalonFX implements PivotIO{
    private final TalonFX pivotMotor;
    private final TalonFXConfiguration pivotConfigs;

    private final StatusSignal<Current> pivotCurrent;
    private final StatusSignal<Temperature> pivotTemp;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Angle> pivotPosition;

    private final MotionMagicVoltage motionMagicVoltageRequest;
    private final VoltageOut voltageOutRequest;

    private double setpointDegrees;
    private double setpointVolts;

    public PivotIOTalonFX(){
        pivotMotor = new TalonFX(canIDConstants.pivotMotor, canIDConstants.canivore);
        pivotConfigs = new TalonFXConfiguration();

        pivotCurrent = pivotMotor.getStatorCurrent();
        pivotTemp = pivotMotor.getDeviceTemp();
        pivotVelocity = pivotMotor.getRotorVelocity();
        pivotVoltage = pivotMotor.getMotorVoltage();
        pivotPosition = pivotMotor.getRotorPosition();

        motionMagicVoltageRequest = new MotionMagicVoltage(0).withEnableFOC(true);
        voltageOutRequest = new VoltageOut(0).withEnableFOC(true);

        setpointDegrees = pivotConstants.stowAngleDegrees;
        setpointVolts = 0;
        
        pivotConfigs.CurrentLimits.StatorCurrentLimit = pivotConstants.statorCurrentLimit;
        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        pivotConfigs.MotorOutput.Inverted = pivotConstants.pivotInvert;
        pivotConfigs.MotorOutput.NeutralMode = pivotConstants.pivotNeutralMode;

        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = pivotConstants.cruiseVelocity;
        pivotConfigs.MotionMagic.MotionMagicAcceleration = pivotConstants.acceleration;
        pivotConfigs.MotionMagic.MotionMagicJerk = pivotConstants.jerk;

        pivotConfigs.Slot0.kP = pivotConstants.kP;
        pivotConfigs.Slot0.kI = pivotConstants.kI;
        pivotConfigs.Slot0.kD = pivotConstants.kD;
        pivotConfigs.Slot0.kS = pivotConstants.kS;
        pivotConfigs.Slot0.kV = pivotConstants.kV;
        pivotConfigs.Slot0.kG = pivotConstants.kG;
        pivotConfigs.Slot0.GravityType = pivotConstants.gravityType;

        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Conversions.DegreesToRotations(pivotConstants.maxAngleDegrees, pivotConstants.gearRatio);
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Conversions.DegreesToRotations(pivotConstants.minAngleDegrees, pivotConstants.gearRatio);

        BaseStatusSignal.setUpdateFrequencyForAll(50,
        pivotCurrent,
        pivotTemp,
        pivotVelocity,
        pivotVoltage,
        pivotPosition);

        pivotMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(PivotIOInputs inputs){
        BaseStatusSignal.refreshAll(pivotCurrent,
        pivotTemp,
        pivotVelocity,
        pivotVoltage,
        pivotPosition);

        inputs.appliedVolts = voltageOutRequest.Output;
        inputs.setpointVolts = setpointVolts;
        inputs.setpointDegrees = setpointDegrees;
        inputs.pivotAngleDegrees = Conversions.RotationsToDegrees(pivotPosition.getValueAsDouble(), pivotConstants.gearRatio);
        inputs.velocityRPS = pivotVelocity.getValueAsDouble();
        inputs.currentAmps = pivotCurrent.getValueAsDouble();
        inputs.tempFahrenheit = pivotTemp.getValueAsDouble();
    }

    @Override
    public void requestVoltage(double volts){
        this.setpointVolts = volts;
        pivotMotor.setControl(voltageOutRequest.withOutput(volts));
    }

    @Override
    public void requestMotionMagic(double degrees){
        this.setpointDegrees = degrees;
        pivotMotor.setControl(motionMagicVoltageRequest.withPosition(Conversions.DegreesToRotations(degrees, pivotConstants.gearRatio)));
    }                                     

    @Override
    public void zeroSensor(double newValue){
        pivotMotor.setPosition(newValue);
    }
}
