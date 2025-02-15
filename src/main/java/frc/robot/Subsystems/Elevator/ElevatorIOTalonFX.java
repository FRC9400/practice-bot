package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import frc.commons.Conversions;
import frc.commons.LoggedTunableNumber;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.elevatorConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    /* Motor Objects */
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFXConfiguration motorConfigs;

    /* Status Signals */
    private final StatusSignal<Current> leftElevatorCurrent;
    private final StatusSignal<Current> rightElevatorCurrent;
    private final StatusSignal<Temperature> leftElevatorTemp;
    private final StatusSignal<Temperature> rightElevatorTemp;
    private final StatusSignal<AngularVelocity> leftElevatorAngularVelocity;
    private final StatusSignal<AngularVelocity> rightElevatorAngularVelocity;
    private final StatusSignal<Angle> leftElevatorPos;

    /* LoggedTunableNumbers */
    LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0);
    LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0);
    LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0);
    LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0);
    LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG",0); 
    LoggedTunableNumber CruiseVelocity = new LoggedTunableNumber( "Elevator/MMCruiseVelocity",10); 
    LoggedTunableNumber Acceleration = new LoggedTunableNumber( "Elevator/MMAcceleration",20); 
    LoggedTunableNumber Jerk = new LoggedTunableNumber("Elevator/MMJerk",5000); //10000 was used on 2024
    
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
        motorConfigs = new TalonFXConfiguration();
        
        /* Status Signals */
        leftElevatorCurrent = leftMotor.getStatorCurrent();
        rightElevatorCurrent = rightMotor.getStatorCurrent();
        leftElevatorTemp = leftMotor.getDeviceTemp();
        rightElevatorTemp = rightMotor.getDeviceTemp();
        leftElevatorAngularVelocity = leftMotor.getRotorVelocity();
        rightElevatorAngularVelocity = rightMotor.getRotorVelocity();
        leftElevatorPos = leftMotor.getRotorPosition();

        /* Control Requests */
        motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true);
        voltageOutRequest = new VoltageOut(0).withEnableFOC(true);

        /* Doubles */
        setpointMeters = 0;
        setpointVolts = 0;

        /* Current Limit Configuration */
        motorConfigs.CurrentLimits.StatorCurrentLimit = elevatorConstants.statorCurrentLimit;
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Motor Output Configuration */
        motorConfigs.MotorOutput.NeutralMode = elevatorConstants.elevatorNeutralMode;
        motorConfigs.MotorOutput.Inverted = elevatorConstants.elevatorMotorInvert;
        
        /* Motion Magic Configuration */
        motorConfigs.MotionMagic.MotionMagicCruiseVelocity = CruiseVelocity.get();
        motorConfigs.MotionMagic.MotionMagicAcceleration = Acceleration.get();
        motorConfigs.MotionMagic.MotionMagicJerk = Jerk.get();

        /* Slot 0 Configuration */
        motorConfigs.Slot0.kP = kP.get();
        motorConfigs.Slot0.kD = kD.get();
        motorConfigs.Slot0.kS = kS.get();
        motorConfigs.Slot0.kV = kV.get();
        motorConfigs.Slot0.kG = kG.get();
        motorConfigs.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        /* Configure Right Motor: Follower */
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));
        
        /* Configure Left Motor: Configs */
        leftMotor.getConfigurator().apply(motorConfigs);

        /* Set Frequency */
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftElevatorCurrent,
            rightElevatorCurrent,
            leftElevatorTemp,
            rightElevatorTemp,
            leftElevatorAngularVelocity,
            rightElevatorAngularVelocity,
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
            leftElevatorPos 
        );
        
        /* Refresh Inputs */
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

    public void zeroSensor(){
        leftMotor.setPosition(0);
    }
}
