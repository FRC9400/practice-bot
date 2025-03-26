package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.commons.Conversions;
import frc.commons.LoggedTunableNumber;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.intakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
    /* Motor Objects */
    private final TalonFX pivotMotor;
    private final TalonFX rollerMotor;
    private final TalonFXConfiguration pivotConfigs;
    private final TalonFXConfiguration rollerConfigs;

    /* Status Signals */
    private final StatusSignal<Current> pivotCurrent;
    private final StatusSignal<Current> rollerCurrent;
    private final StatusSignal<Temperature> pivotTemp;
    private final StatusSignal<Temperature> rollerTemp;
    private final StatusSignal<AngularVelocity> pivotAngularVelocity;
    private final StatusSignal<AngularVelocity> rollerAngularVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Angle> pivotAngle;

    /* Control Requests */
    private MotionMagicVoltage pivotMotionMagicRequest;
    private VoltageOut pivotVoltageOutRequest;
    private VoltageOut rollerVoltageOutRequest;

    /* Doubles */
    private double pivotSetpointDeg;
    private double pivotSetpointVolts;
    private double rollerSetpointVolts;

    LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/kP", 0);
    LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/kD", 0);
    LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/kS", 0);
    LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/kV", 0);
    LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/kG",0); 
    LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber( "Pivot/kMotionCruiseVelocity",10);
    LoggedTunableNumber kMotionAcceleration = new LoggedTunableNumber( "Pivot/kMotionAcceleration",20);
    LoggedTunableNumber kMotionJerk = new LoggedTunableNumber("Pivot/kMotionJerk",10000);

    public IntakeIOTalonFX(){
        /* Motor Objects */
        pivotMotor = new TalonFX(canIDConstants.pivotMotor, canIDConstants.rio);
        rollerMotor = new TalonFX(canIDConstants.rollerMotor, canIDConstants.rio);
        pivotConfigs = new TalonFXConfiguration();
        rollerConfigs = new TalonFXConfiguration();

        /* Status Signals */
        pivotCurrent = pivotMotor.getStatorCurrent();
        rollerCurrent = rollerMotor.getStatorCurrent();
        pivotTemp = pivotMotor.getDeviceTemp();
        rollerTemp = rollerMotor.getDeviceTemp();
        pivotAngularVelocity = pivotMotor.getRotorVelocity();
        rollerAngularVelocity = rollerMotor.getRotorVelocity();
        pivotVoltage = pivotMotor.getMotorVoltage();
        rollerVoltage = rollerMotor.getMotorVoltage();
        pivotAngle = pivotMotor.getRotorPosition();
        
        /* Control Requests */
        pivotMotionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true);
        pivotVoltageOutRequest = new VoltageOut(0).withEnableFOC(true);
        rollerVoltageOutRequest = new VoltageOut(0).withEnableFOC(true);

        /* Doubles */
        pivotSetpointDeg = 0;
        pivotSetpointVolts = 0;
        rollerSetpointVolts = 0;

        /* Current Limit Configuration */
        rollerConfigs.CurrentLimits.StatorCurrentLimit = intakeConstants.rollerCurrentLimit;
        rollerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Motor Output Configuration */
        rollerConfigs.MotorOutput.Inverted = intakeConstants.rollerInvert;
        pivotMotor.setPosition(0);

        rollerMotor.getConfigurator().apply(rollerConfigs);
        
        /* Set Frequency */
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            pivotCurrent,
            rollerCurrent,
            pivotTemp,
            rollerTemp,
            pivotAngularVelocity,
            rollerAngularVelocity,
            pivotVoltage,
            rollerVoltage,
            pivotAngle
        );

        /* Optimize Bus Utilization */
        rollerMotor.optimizeBusUtilization();
        pivotMotor.optimizeBusUtilization();
    }

    public void updateInputs(IntakeIOInputs inputs){
        /* Refresh Status Signals */
        BaseStatusSignal.refreshAll(
            pivotCurrent,
            rollerCurrent,
            pivotTemp,
            rollerTemp,
            pivotAngularVelocity,
            rollerAngularVelocity,
            pivotVoltage,
            rollerVoltage,
            pivotAngle    
        );

        /* Refresh Inputs */
        inputs.pivotVoltage = pivotVoltage.getValueAsDouble();
        inputs.pivotSetpointVolts = pivotSetpointVolts;
        inputs.pivotAppliedVolts = pivotVoltageOutRequest.Output;
        inputs.pivotCurrentAmps = pivotCurrent.getValueAsDouble();
        inputs.pivotSetpointDeg = pivotSetpointDeg;
        inputs.pivotSetpointRot = Conversions.DegreesToRotations(pivotSetpointDeg, intakeConstants.pivotGearRatio);
        inputs.pivotPositionDeg = Conversions.RotationsToDegrees(pivotAngle.getValueAsDouble(), intakeConstants.pivotGearRatio);
        inputs.pivotPositionRot = pivotAngle.getValueAsDouble();
        inputs.pivotTempFahrenheit = pivotTemp.getValueAsDouble();
        inputs.pivotVelocityRPS = pivotAngularVelocity.getValueAsDouble();
        
        inputs.rollerAppliedVolts = rollerVoltageOutRequest.Output;
        inputs.rollerSetpointVolts = rollerSetpointVolts;
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
        inputs.rollerVelocityRPS = rollerAngularVelocity.getValueAsDouble();
        inputs.rollerTempFahrenheit = rollerTemp.getValueAsDouble();
        
    }

    public void zeroSensor(){
        pivotMotor.setPosition(0);
    }

    public void requestRollerVoltage(double volts){
        this.rollerSetpointVolts = volts;
        rollerMotor.setControl(rollerVoltageOutRequest.withOutput(volts));
    }

    public void requestPivotVoltage(double volts){
        this.pivotSetpointVolts = volts;
        pivotMotor.setControl(pivotVoltageOutRequest.withOutput(volts));
    }

    public void requestMotionMagic(double degrees){
        this.pivotSetpointDeg = degrees;
        pivotMotor.setControl(pivotMotionMagicRequest.withPosition(Conversions.DegreesToRotations(degrees, intakeConstants.pivotGearRatio)));
    }


    public void updateTunableNumbers() {
        if (
          kD.hasChanged(kD.hashCode()) ||
          kG.hasChanged(kG.hashCode()) ||
          kS.hasChanged(kS.hashCode()) ||
          kP.hasChanged(kP.hashCode()) ||
          kV.hasChanged(kV.hashCode())||
          kMotionAcceleration.hasChanged(kMotionAcceleration.hashCode()) ||
          kMotionCruiseVelocity.hasChanged(kMotionCruiseVelocity.hashCode()) ||
          kMotionJerk.hasChanged(kMotionJerk.hashCode())
        ) {
            pivotConfiguration();
        }
      }

    public void pivotConfiguration(){
        pivotConfigs.CurrentLimits.StatorCurrentLimit = intakeConstants.pivotCurrentLimit;
        pivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        
        pivotConfigs.MotorOutput.NeutralMode = intakeConstants.pivotNeutralMode;
        pivotConfigs.MotorOutput.Inverted = intakeConstants.pivotInvert;

        pivotConfigs.Slot0.kP = kP.get();
        pivotConfigs.Slot0.kD = kD.get();
        pivotConfigs.Slot0.kS = kS.get();
        pivotConfigs.Slot0.kV = kV.get();
        pivotConfigs.Slot0.kG = kG.get();
        pivotConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConfigs.MotionMagic.MotionMagicCruiseVelocity = kMotionCruiseVelocity.get();
        pivotConfigs.MotionMagic.MotionMagicAcceleration = kMotionAcceleration.get();
        pivotConfigs.MotionMagic.MotionMagicJerk = kMotionJerk.get();


        pivotMotor.getConfigurator().apply(pivotConfigs);
    }

}