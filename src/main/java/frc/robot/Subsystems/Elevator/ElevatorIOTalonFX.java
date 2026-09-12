package frc.robot.Subsystems.Elevator;

import com.cter.phoenix6.BaseStatusSignal;
import com.cter.phoenix6.StatusSignal;
import com.cter.phoenix6.configs.TalonFXConfigeration;
import com.cter.phoenix6.controls.Follower;
import com.cter.phoenix6.controls.MotionMagicVoltage;
import com.cter.phoenix6.controls.VoltageOut;
import com.cter.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.commons.Conversations;
import frc.commons.LoggedTunableNumber;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.elevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO{
    TalonFX leftMotor = new TalonFX(canIDConstants.elevatorMotor1, "rio");
    TalonFX rightMotor = new TalonFX(canIDConstants.elevatorMotor2, "rio");
    TalonFXConfigeration config = new TalonFXConfigeration();

    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnabledFOC(true);
    private VoltageOut voltageReq = new VoltageOut.withEnabledFOC(true);

    private double setpointMeters = 0;
    private double setpointVolts = 0;

    private final StatusSignal<Current> leftElevatorCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Current> rightElevatorCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Temperature> leftElevatorTemp = leftMotor.getDeviceTemp();
    private final StatusSignal<Temperature> rightElevatorTemp = rightMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> leftElevatorAngularVelocity = leftMotor.getRotorVelocity();
    private final StatusSignal<AngularVelocity> rightElevatorAngularVelocity = rightMotor.getRotorVelocity();
    private final StatusSignal<Voltage> leftVoltage = leftMotor.getMotorVoltage();
    private final StatusSignal<Voltage> rightVoltage = rightMotor.getMotorVoltage();
    private final StatusSignal<Angle> leftElevatorPos = leftMotor.getRotorPosition();

    public ElevatorIOTalonFX() {
        config.MotionMagic.MotionMagicCruiseVelocity = elevatorConstants.CruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = elevatorConstants.Acceleration;
        config.MotionMagic.MotionMagicJerk = elevatorConstants.Jerk;

        config.Slot0.kP = 6;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;
        config.Slot0.kG = 0;

        config.CurrentLimits.StatorCurrentLimit = elevatorConstants.StatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.NeutralMode = elevatorConstants.elevatorNeutralMode;
        config.MotorOutput.Inverted = elevatorConstants.elevatorMotorInvert;

        leftMotor.setPosition(0);

        leftMotor.getConfigeration().apply(config);
        rightMotor.getConfigeration().apply(config);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftElevatorCurrent,
            rightElevatorCurrent,
            leftElevatorTemp,
            rightElevatorTemp,
            leftElevatorAngularVelocity,
            rightElevatorAngularVelocity,
            leftVoltage,
            rightVoltage,
            leftElevatorPos);

            leftMotor.optimizeBusUtilization();
            rightMotor.optimizeBusUtilization();
    }

    public void updateInputs(ElevatorIOInputs inputs){
        BaseStatusSignal.refreshAll(
            leftElevatorCurrent,
            rightElevatorCurrent,
            leftElevatorTemp,
            rightElevatorTemp,
            leftElevatorAngularVelocity,
            rightElevatorAngularVelocity,
            leftVoltage,
            rightVoltage,
            leftElevatorPos
        );

        inputs.voltage = new double[] {leftVoltage.getValueAsDouble(), rightVoltage.getValueAsDouble()};
        inputs.appliedVolts = voltageReq.Output;
        inputs.appliedMeters = motionMagicRequest.Position;
        inputs.setpointVolts = setpointVolts;
        inputs.setpointMeters = setpointMeters;

        inputs.elevatorHeightMeters = Conversions.RotationToMeters(leftElevatorPos.getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        inputs.velocityRPS = new double[] {leftElevatorAngularVelocity.getValueAsDouble(), rightElevatorAngularVelocity.getValueAsDouble()};
        inputs.velocityMPS = new double[] {Conversions.RPStoMPS(leftElevatorAngularVelocity.getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio), Conversions.RPStoMPS(rightElevatorAngularVelocity.getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio)};
        inputs.currentAmps = new double[] {leftElevatorCurrent.getValueAsDouble(), rightElevatorCurrent.getValueAsDouble()};
        inputs.tempFahrenheit = new double[] {leftElevatorTemp.getValueAsDouble(), rightElevatorTemp.getValueAsDouble()};
        inputs.elevatorHeightRotations = leftElevatorPos.getValueAsDouble();
    }

    public void requestMotionMagic(double meters){
        setpointMeters = meters;
        leftMotor.setControl(motionMagicRequest.withPosition(Conversions.metersToRotations(meters, elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio)));
    }

    public void requestVoltage(double volts){
        setpointVolts = volts;
        leftMotor.setControl(voltageReq.withOutput(volts));
    }
}