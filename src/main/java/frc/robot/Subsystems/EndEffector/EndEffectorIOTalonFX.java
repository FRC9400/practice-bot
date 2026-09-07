package frc.robot.Subsystems.EndEffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.endEffectorConstants;

public class EndEffectorIOTalonFX implements EndEffectorIO {
    /* Motor Objects */
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfigs;

    /* Status Signals */
    private final StatusSignal<Current> endEffectorCurrent;
    private final StatusSignal<Temperature> endEffectorTemp;
    private final StatusSignal<AngularVelocity> endEffectorAngularVelocity;
    private final StatusSignal<Voltage> voltage;
    
    /* Control Requests */
    private VoltageOut voltageOutRequest;

    /* Doubles */
    private double setpointVolts;

    public EndEffectorIOTalonFX(){
        /* Motor Objects */
        motor = new TalonFX(canIDConstants.endEffectorMotor, canIDConstants.rio);
        motorConfigs = new TalonFXConfiguration();

        /* Status Signals */
        endEffectorCurrent = motor.getStatorCurrent();
        endEffectorTemp = motor.getDeviceTemp();
        endEffectorAngularVelocity = motor.getRotorVelocity();
        voltage = motor.getMotorVoltage();
    
        /* Control Requests */
        voltageOutRequest = new VoltageOut(0).withEnableFOC(true);

        /* Doubles */
        setpointVolts = 0;

        /* Current Limit Configuration */
        motorConfigs.CurrentLimits.StatorCurrentLimit = endEffectorConstants.statorCurrentLimit;
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Motor Output Configuration */
        motorConfigs.MotorOutput.Inverted = endEffectorConstants.endEffectorMotorInvert;

        /* Configure Motors */
        motor.getConfigurator().apply(motorConfigs);

        /* Set Frequency */
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            endEffectorCurrent,
            endEffectorTemp,
            endEffectorAngularVelocity,
            voltage);

        /* Optimize Bus Utilization */
        motor.optimizeBusUtilization();
    }

    public void updateInputs(EndEffectorInputs inputs){
        /* Refresh Status Signals */
        BaseStatusSignal.refreshAll(
            endEffectorCurrent,
            endEffectorTemp,
            endEffectorAngularVelocity,
            voltage  
        );

        /* Refresh Inputs */
        inputs.appliedVolts = voltageOutRequest.Output;
        inputs.setpointVolts = setpointVolts;
        inputs.voltage = voltage.getValueAsDouble();
        inputs.velocityRPS = endEffectorAngularVelocity.getValueAsDouble();
        inputs.currentAmps = endEffectorCurrent.getValueAsDouble();
        inputs.tempFahrenheit = endEffectorTemp.getValueAsDouble();
    }

    public void requestVoltage(double volts){
        this.setpointVolts = volts;
        motor.setControl(voltageOutRequest.withOutput(volts));
    }
}
