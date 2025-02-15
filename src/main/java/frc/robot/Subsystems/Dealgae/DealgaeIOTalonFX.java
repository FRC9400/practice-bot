package frc.robot.Subsystems.Dealgae;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.dealgaeConstants;

public class DealgaeIOTalonFX implements DealgaeIO {
    /* Motor Objects */
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfigs;

    /* Status Signals */
    private final StatusSignal<Current> dealgaeCurrent;
    private final StatusSignal<Temperature> dealgaeTemp;
    private final StatusSignal<AngularVelocity> dealgaeAngularVelocity;

    /* Control Requests */
    private VoltageOut voltageOutRequest;
    
    /* Doubles */
    private double setpointVolts;

    public DealgaeIOTalonFX(){
        /* Motor Objects */
        motor = new TalonFX(canIDConstants.dealgaeMotor, canIDConstants.rio);
        motorConfigs = new TalonFXConfiguration();

        /* Status Signals */
        dealgaeCurrent = motor.getStatorCurrent();
        dealgaeTemp = motor.getDeviceTemp();
        dealgaeAngularVelocity = motor.getRotorVelocity();

        /* Control Requests */
        voltageOutRequest = new VoltageOut(0).withEnableFOC(true);

        /* Doubles */
        setpointVolts = 0;

        /* Current Limit Configuration */
        motorConfigs.CurrentLimits.StatorCurrentLimit = dealgaeConstants.statorCurrentLimit;
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Motor Output Configuration */
        motorConfigs.MotorOutput.Inverted = dealgaeConstants.deAlgaeMotorInvert;

        /* Configure Motors */
        motor.getConfigurator().apply(motorConfigs);

        /* Set Frequency */
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            dealgaeCurrent,
            dealgaeTemp,
            dealgaeAngularVelocity);
        
        /* Optimize Bus Utilization */
        motor.optimizeBusUtilization();
    }

    public void updateInputs(DealgaeIOInputs inputs){
        /* Refresh Status Signals */
        BaseStatusSignal.refreshAll(
            dealgaeCurrent,
            dealgaeTemp,
            dealgaeAngularVelocity);
        
        /* Refresh Inputs */
        inputs.appliedVolts = voltageOutRequest.Output;
        inputs.setpointVolts = setpointVolts;
        inputs.velocityRPS = dealgaeAngularVelocity.getValueAsDouble();
        inputs.currentAmps = dealgaeCurrent.getValueAsDouble();
        inputs.tempFahrenheit = dealgaeTemp.getValueAsDouble();
    }

    public void requestVoltage(double volts){
        this.setpointVolts = volts;
        motor.setControl(voltageOutRequest.withOutput(volts));
    }

}
