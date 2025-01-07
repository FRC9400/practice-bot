package frc.robot.Subsystems.OTB_Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import frc.robot.Constants.otbIntakeConstants;
import frc.robot.Constants.canIDConstants;

public class OTB_IntakeIOTalonFX implements OTB_IntakeIO {
    private final TalonFX pivot = new TalonFX(canIDConstants.otbIntakePivotMotor, "canivore");
    private final TalonFX intake = new TalonFX(canIDConstants.otbIntakeMotor, "canivore");

    private final TalonFXConfiguration pivotConfigs;
    private final TalonFXConfiguration intakeConfigs;

    MotionMagicVoltage pivotMotionMagicRequest;
    VoltageOut pivotVoltageRequest;
    VoltageOut intakeVoltageRequest;

    double pivotSetpoint;
    double intakeSetpointVolts;

    private final StatusSignal<Current> pivotCurrent = pivot.getStatorCurrent();
    private final StatusSignal<Temperature> pivotTemp = pivot.getDeviceTemp();
    private final StatusSignal<AngularVelocity> pivotRPS = pivot.getRotorVelocity();
    private final StatusSignal<Angle> pivotPos = pivot.getRotorPosition();

    private final StatusSignal<Current> intakeCurrent = intake.getStatorCurrent();
    private final StatusSignal<Temperature> intakeTemp = intake.getDeviceTemp();
    private final StatusSignal<AngularVelocity> intakeRPS = intake.getRotorVelocity();

    
    public OTB_IntakeIOTalonFX() {

        pivotConfigs = new TalonFXConfiguration();
        intakeConfigs = new TalonFXConfiguration();

        var pivotMotorOuputConfigs = pivotConfigs.MotorOutput;
        pivotMotorOuputConfigs.NeutralMode = NeutralModeValue.Brake;
        pivotMotorOuputConfigs.Inverted = otbIntakeConstants.pivotInvert;

        var pivotCurrentLimitConfigs = pivotConfigs.CurrentLimits;
        pivotCurrentLimitConfigs.StatorCurrentLimit = otbIntakeConstants.pivotCurrentLimit;
        pivotCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        var slot0Configs = pivotConfigs.Slot0;
        slot0Configs.kP = 1;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;
        slot0Configs.kS = 0.20835;
        slot0Configs.kV = 0.0084435;
        slot0Configs.kA = 0.001;
        slot0Configs.kG = 0.48633;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfigs = pivotConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 60.0;
        motionMagicConfigs.MotionMagicAcceleration = 120.0;
        motionMagicConfigs.MotionMagicJerk = 10000.0;

        var feedbackConfigs = pivotConfigs.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        pivot.setPosition(0);

        var intakeMotorOutputConfigs = intakeConfigs.MotorOutput;
        intakeMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
        intakeMotorOutputConfigs.Inverted = otbIntakeConstants.intakeInvert;

        var intakeCurrentLimitConfigs = intakeConfigs.CurrentLimits;
        intakeCurrentLimitConfigs.StatorCurrentLimit = otbIntakeConstants.intakeCurrentLimit;
        intakeCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        pivotVoltageRequest = new VoltageOut(0).withEnableFOC(true);
        intakeVoltageRequest = new VoltageOut(0).withEnableFOC(true);

        pivot.getConfigurator().apply(pivotConfigs);
        intake.getConfigurator().apply(intakeConfigs);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                pivotCurrent,
                pivotPos,
                pivotRPS,
                pivotTemp,
                intakeTemp,
                intakeCurrent,
                intakeRPS
               );

        intake.optimizeBusUtilization();
    }

    public void updateInputs(OTB_IntakeIOInputs otbIntakeInputs){
        BaseStatusSignal.refreshAll(
          pivotCurrent,
            pivotPos,
            pivotRPS,
            pivotTemp,
            intakeTemp,
            intakeCurrent,
            intakeRPS
        );
        otbIntakeInputs.pivotAppliedVolts = pivotVoltageRequest.Output;
        otbIntakeInputs.pivotCurrent = pivotCurrent.getValueAsDouble();
        otbIntakeInputs.pivotPosDeg = Conversions.RotationsToDegrees(pivotPos.getValueAsDouble(), otbIntakeConstants.gearRatio);
        otbIntakeInputs.pivotPosRot = pivotPos.getValueAsDouble();
        otbIntakeInputs.pivotSetpointDeg = pivotSetpoint;
        otbIntakeInputs.pivotSetpointRot = Conversions.DegreesToRotations(pivotSetpoint, otbIntakeConstants.gearRatio);
        otbIntakeInputs.pivotTemperature = pivotTemp.getValueAsDouble();
        otbIntakeInputs.pivotRPS = pivotRPS.getValueAsDouble();

        otbIntakeInputs.intakeTemperature = intakeTemp.getValueAsDouble();
        otbIntakeInputs.intakeAppliedVolts = intakeVoltageRequest.Output;
        otbIntakeInputs.intakeCurrent = intakeCurrent.getValueAsDouble();
        otbIntakeInputs.intakeRPS = intakeRPS.getValueAsDouble();
        otbIntakeInputs.intakeSetpointVolts = this.intakeSetpointVolts;

    }

    public void requestPivotVoltage(double voltage) {
        pivot.setControl(pivotVoltageRequest.withOutput(voltage));
    }

    public void requestSetpoint(double angleDegrees) {
        this.pivotSetpoint = angleDegrees;
        double pivotSetpointRotations = Conversions.DegreesToRotations(angleDegrees, otbIntakeConstants.gearRatio);
        pivot.setControl(pivotMotionMagicRequest.withPosition(pivotSetpointRotations));
    }

    public void requestIntakeVoltage(double voltage) {
        this.intakeSetpointVolts = voltage;
        intake.setControl(intakeVoltageRequest.withOutput(intakeSetpointVolts));
    }

    public void zeroPosition(){
        pivot.setPosition(0);
    }
}