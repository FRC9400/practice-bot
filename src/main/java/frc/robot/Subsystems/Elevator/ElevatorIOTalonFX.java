package frc.robot.Subsystems.Elevator;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.elevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    /* Motor Objects */
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFXConfiguration leftConfigs;
    private final TalonFXConfiguration rightConfigs;

    /* Status Signals */
    private final StatusSignal<Current> leftElevatorCurrent;
    private final StatusSignal<Current> rightElevatorCurrent;
    private final StatusSignal<Temperature> leftElevatorTemp;
    private final StatusSignal<Temperature> rightElevatorTemp;
    private final StatusSignal<AngularVelocity> leftElevatorAngularVelocity;
    private final StatusSignal<AngularVelocity> rightElevatorAngularVelocity;
    private final StatusSignal<Voltage> leftVoltage;
    private final StatusSignal<Voltage> rightVoltage;

    private final StatusSignal<Angle> leftElevatorPos;
    
    /* Control Requests */
    private MotionMagicVoltage motionMagicRequest;
    private VoltageOut voltageOutRequest;

    /* Doubles */
    private double setpointMeters;
    private double setpointVolts;
    
    public ElevatorIOTalonFX(){
        /* Motor Objects */
        leftMotor = new TalonFX(canIDConstants.leftElevatorMotor, canIDConstants.canivore);
        rightMotor = new TalonFX(canIDConstants.rightElevatorMotor, canIDConstants.canivore);
        leftConfigs = new TalonFXConfiguration();
        rightConfigs = new TalonFXConfiguration();
        
        /* Status Signals */
        leftElevatorCurrent = leftMotor.getStatorCurrent();
        rightElevatorCurrent = rightMotor.getStatorCurrent();
        leftElevatorTemp = leftMotor.getDeviceTemp();
        rightElevatorTemp = rightMotor.getDeviceTemp();
        leftElevatorAngularVelocity = leftMotor.getRotorVelocity();
        rightElevatorAngularVelocity = rightMotor.getRotorVelocity();
        leftVoltage = leftMotor.getMotorVoltage();
        rightVoltage = rightMotor.getMotorVoltage();
        leftElevatorPos = leftMotor.getRotorPosition();

        /* Control Requests */
        motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true);
        voltageOutRequest = new VoltageOut(0).withEnableFOC(true);

        /* Doubles */
        setpointMeters = 0;
        setpointVolts = 0;

        /* Current Limit Configuration */
        leftConfigs.CurrentLimits.StatorCurrentLimit = elevatorConstants.statorCurrentLimit;
        leftConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rightConfigs.CurrentLimits.StatorCurrentLimit = elevatorConstants.statorCurrentLimit;
        rightConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Motor Output Configuration */
        leftConfigs.MotorOutput.NeutralMode = elevatorConstants.elevatorNeutralMode;
        rightConfigs.MotorOutput.NeutralMode = elevatorConstants.elevatorNeutralMode;
        leftConfigs.MotorOutput.Inverted = elevatorConstants.elevatorMotorInvert;
        rightConfigs.MotorOutput.Inverted = elevatorConstants.elevatorMotorInvert;
        
        /* Motion Magic Configuration */
        leftConfigs.MotionMagic.MotionMagicCruiseVelocity = 300;
        leftConfigs.MotionMagic.MotionMagicAcceleration = 450;
        leftConfigs.MotionMagic.MotionMagicJerk = 10000;

        /* Slot 0 Configuration */
        leftConfigs.Slot0.kP = 2;
        leftConfigs.Slot0.kD = 0;
        leftConfigs.Slot0.kS = 0.088822;
        leftConfigs.Slot0.kV = 0;
        leftConfigs.Slot0.kG = 0.464;
        leftConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        
        leftMotor.setPosition(Conversions.metersToRotations(Units.inchesToMeters(0), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio));
        /* Configure Right Motor: Follower */
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));
        
        /* Configure Configs */
        leftMotor.getConfigurator().apply(leftConfigs);
        rightMotor.getConfigurator().apply(rightConfigs);
       
        /* Set Frequency */
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

        /* Optimize Bus Utilization */
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    public void updateInputs(ElevatorIOInputs inputs){
        /* Refresh Status Signals */
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
        
        /* Refresh Inputs */
        inputs.voltage = new double[] {leftVoltage.getValueAsDouble(), rightVoltage.getValueAsDouble()};
        inputs.appliedVolts = voltageOutRequest.Output;
        inputs.appliedMeters = motionMagicRequest.Position;
        inputs.setpointVolts = setpointVolts;
        inputs.setpointMeters = setpointMeters;
        
        inputs.elevatorHeightMeters = Conversions.RotationsToMeters(leftElevatorPos.getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio);
        inputs.velocityRPS = new double[] {leftElevatorAngularVelocity.getValueAsDouble(), rightElevatorAngularVelocity.getValueAsDouble()};
        inputs.velocityMPS =  new double[] {Conversions.RPStoMPS(leftElevatorAngularVelocity.getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio), Conversions.RPStoMPS(rightElevatorAngularVelocity.getValueAsDouble(), elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio)};
        inputs.currentAmps = new double[] {leftElevatorCurrent.getValueAsDouble(), rightElevatorCurrent.getValueAsDouble()};
        inputs.tempFahrenheit = new double[] {leftElevatorTemp.getValueAsDouble(), rightElevatorTemp.getValueAsDouble()};
    }

    public void requestVoltage(double volts){
        this.setpointVolts = volts;
        leftMotor.setControl(voltageOutRequest.withOutput(volts));
    }

    public void requestMotionMagic(double meters){
        this.setpointMeters = meters;
        leftMotor.setControl(motionMagicRequest.withPosition(Conversions.metersToRotations(meters, elevatorConstants.wheelCircumferenceMeters, elevatorConstants.gearRatio)));
    }

    public void zeroSensor(double newValue){
        leftMotor.setPosition(newValue);
    }

    public void resetMotionMagicConfigs(boolean down){
        MotionMagicConfigs configs  = new MotionMagicConfigs();
        if(down){
            configs.MotionMagicCruiseVelocity = 60;
            configs.MotionMagicAcceleration = 120;
            configs.MotionMagicJerk = 10000;
        }else{
            configs.MotionMagicCruiseVelocity = 300;
            configs.MotionMagicAcceleration = 450;
           configs.MotionMagicJerk = 10000;
        }
        leftMotor.getConfigurator().apply(configs);
    }
}