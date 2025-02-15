package frc.robot.Subsystems.EndEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.endEffectorConstants;

public class EndEffectorIOTalonFX implements EndEffectorIO {
    /* Motor Objects */
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final TalonFXConfiguration leftMotorConfigs;
    private final TalonFXConfiguration rightMotorConfigs;

    /* Status Signals */
    private final StatusSignal<Current> leftEndEffectorCurrent;
    private final StatusSignal<Current> rightEndEffectorCurrent;
    private final StatusSignal<Temperature> leftEndEffectorTemp;
    private final StatusSignal<Temperature> rightEndEffectorTemp;
    private final StatusSignal<AngularVelocity> leftEndEffectorAngularVelocity;
    private final StatusSignal<AngularVelocity> rightEndEffectorAngularVelocity;
    
    /* Control Requests */
    private VoltageOut leftVoltageOutRequest;
    private VoltageOut rightVoltageOutRequest;

    /* Doubles */
    private double leftSetpointVolts;
    private double rightSetpointVolts;

    public EndEffectorIOTalonFX(){
        /* Motor Objects */
        leftMotor = new TalonFX(canIDConstants.leftEndEffectorMotor, canIDConstants.rio);
        rightMotor = new TalonFX(canIDConstants.rightEndEffectorMotor, canIDConstants.rio);
        leftMotorConfigs = new TalonFXConfiguration();
        rightMotorConfigs = new TalonFXConfiguration();

        /* Status Signals */
        leftEndEffectorCurrent = leftMotor.getStatorCurrent();
        rightEndEffectorCurrent = rightMotor.getStatorCurrent();
        leftEndEffectorTemp = leftMotor.getDeviceTemp();
        rightEndEffectorTemp = rightMotor.getDeviceTemp();
        leftEndEffectorAngularVelocity = leftMotor.getRotorVelocity();
        rightEndEffectorAngularVelocity = rightMotor.getRotorVelocity();
    
        /* Control Requests */
        leftVoltageOutRequest = new VoltageOut(0).withEnableFOC(true);
        rightVoltageOutRequest = new VoltageOut(0).withEnableFOC(true);

        /* Doubles */
        leftSetpointVolts = 0;
        rightSetpointVolts = 0;

        /* Current Limit Configuration */
        leftMotorConfigs.CurrentLimits.StatorCurrentLimit = endEffectorConstants.statorCurrentLimit;
        leftMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        rightMotorConfigs.CurrentLimits.StatorCurrentLimit = endEffectorConstants.statorCurrentLimit;
        rightMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Motor Output Configuration */
        leftMotorConfigs.MotorOutput.Inverted = endEffectorConstants.leftEndEffectorMotorInvert;
        rightMotorConfigs.MotorOutput.Inverted = endEffectorConstants.rightEndEffectorMotorInvert;

        /* Configure Motors */
        leftMotor.getConfigurator().apply(leftMotorConfigs);
        rightMotor.getConfigurator().apply(rightMotorConfigs);

        /* Set Frequency */
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftEndEffectorCurrent,
            rightEndEffectorCurrent,
            leftEndEffectorTemp,
            rightEndEffectorTemp,
            leftEndEffectorAngularVelocity,
            rightEndEffectorAngularVelocity);

        /* Optimize Bus Utilization */
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    public void updateInputs(EndEffectorInputs inputs){
        /* Refresh Status Signals */
        BaseStatusSignal.refreshAll(
            leftEndEffectorCurrent,
            rightEndEffectorCurrent,
            leftEndEffectorTemp,
            rightEndEffectorTemp,
            leftEndEffectorAngularVelocity,
            rightEndEffectorAngularVelocity
        );

        /* Refresh Inputs */
        inputs.appliedVolts = new double[] {leftVoltageOutRequest.Output, rightVoltageOutRequest.Output};
        inputs.setpointVolts = new double[] {leftSetpointVolts, rightSetpointVolts};
        inputs.velocityRPS = new double[] {leftEndEffectorAngularVelocity.getValueAsDouble(), rightEndEffectorAngularVelocity.getValueAsDouble()};
        inputs.currentAmps = new double[] {leftEndEffectorCurrent.getValueAsDouble(), rightEndEffectorCurrent.getValueAsDouble()};
        inputs.tempFahrenheit = new double[] {leftEndEffectorTemp.getValueAsDouble(), rightEndEffectorTemp.getValueAsDouble()};
    }

    public void requestVoltage(double volts, double ratio){
        this.leftSetpointVolts = volts;
        this.rightSetpointVolts = volts * ratio;
        leftMotor.setControl(leftVoltageOutRequest.withOutput(volts));
        rightMotor.setControl(rightVoltageOutRequest.withOutput(volts * ratio));
    }

}
