package frc.robot.Subsystems.Funnel;

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
import frc.robot.Constants.funnelConstants;

public class FunnelIOTalonFX implements FunnelIO {
    // Motor Objects
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfigs;

    //Status Signals
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;

    private VoltageOut voltageOutRequest; //control request

    private double setpointVolts; //double
    
    public FunnelIOTalonFX(){
        //Motor Objects
        motor = new TalonFX(canIDConstants.funnelMotor, canIDConstants.canivore);
        motorConfigs = new TalonFXConfiguration();

        //Status Signals
        current = motor.getStatorCurrent();
        temperature = motor.getDeviceTemp();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();

        voltageOutRequest = new VoltageOut(0); //control req
        setpointVolts = 0; //double

        //Current Limit Config
        motorConfigs.CurrentLimits.StatorCurrentLimit = funnelConstants.statorCurrentLimit;
        motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        //Motor Output Config (+inv)
        motorConfigs.MotorOutput.Inverted = funnelConstants.funnelMotorInvert;
        motor.getConfigurator().apply(motorConfigs);

        //Frequency
        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            current,
            temperature,
            voltage,
            velocity
        );

        //Optimize bus utilization
        motor.optimizeBusUtilization();
    }

    public void updateInputs(FunnelInputs inputs){
        //Refresh status signals
        BaseStatusSignal.refreshAll(
            current,
            temperature,
            voltage,
            velocity
        );
    }

    //request voltage
    public void requestVoltage(double volts, double ratio){
        this.setpointVolts = volts;
        motor.setControl(voltageOutRequest.withOutput(volts));
    }

    
}
