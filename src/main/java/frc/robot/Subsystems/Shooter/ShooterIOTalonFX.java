package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.endEffectorConstants;


public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leftMotor = new TalonFX(canIDConstants.leftShooterMotor, "rio");
    private final TalonFX rightMotor = new TalonFX(canIDConstants.rightShooterMotor, "rio");
    private TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();

    private VoltageOut shootRequestVoltage = new VoltageOut(0).withEnableFOC(true);
    private VelocityVoltage leftShootRequestVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private final MotionMagicVelocityVoltage leftShootRequestMMVelocity = new MotionMagicVelocityVoltage(0).withEnableFOC(true);

    private final StatusSignal<Current> leftShooterCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Current> rightShooterCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Temperature> leftShooterTemp = leftMotor.getDeviceTemp();
    private final StatusSignal<Temperature> rightShooterTemp = rightMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> leftShooterSpeedRPS = leftMotor.getRotorVelocity();
    private final StatusSignal<AngularVelocity> rightShooterSpeedRPS = rightMotor.getRotorVelocity();
    private final StatusSignal<Voltage> leftVoltage = leftMotor.getMotorVoltage();
    private final StatusSignal<Voltage> rightVoltage = rightMotor.getMotorVoltage();

    private double leftShooterSetPointMPS;

    public void ShooterIOTalonFx(){
        leftMotorConfigs.CurrentLimits.StatorCurrentLimit= 60; //max 60a flow through
        leftMotorConfigs.CurrentLimits.StatorCurrentLimitEnable= true;//switches limit on
        leftMotorConfigs.MotorOutput.Inverted= InvertedValue.CounterClockwise_Positive;

        leftMotorConfigs.CurrentLimits.SupplyCurrentLimit = 60;//current amps
        leftMotorConfigs.CurrentLimits.SupplyCurrentLowerLimit = 40;
        leftMotorConfigs.CurrentLimits.SupplyCurrentLowerTime = 1.0;
        leftMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        //49-52 60 amps 1 sec then 40

        leftMotorConfigs.Voltage.PeakForwardVoltage = 11;
        leftMotorConfigs.Voltage.PeakReverseVoltage = -11;

        leftMotorConfigs.Slot0.kP = 0.29;
        leftMotorConfigs.Slot0.kD = 0.01;
        leftMotorConfigs.Slot0.kS = 0.2;
        leftMotorConfigs.Slot0.kV = 0.115;
        leftMotorConfigs.Slot0.kA = 0;

        leftMotorConfigs.MotionMagic.MotionMagicAcceleration = Conversions.MPStoRPS(15,shooterConstants.wheelCircumferenceMeters, 1);
        leftMotorConfigs.MotionMagic.MotionMagicJerk = Conversions.MPStoRPS(30, shooterConstants.wheelCircumferenceMeters, 1);

        leftMotor.getConfigurator().apply(leftMotorConfigs);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        BaseStatusSignal.setUpdateFrequencyForAll(50, leftShooterCurrent,rightShooterCurrent,leftShooterTemp,rightShooterTemp,leftShooterSpeedRPS,rightShooterSpeedRPS,leftVoltage,rightVoltage);

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    public void requestVelocity(double vel){
        this.leftShooterSetPointMPS = vel;
        leftMotor.setControl(leftShooterRequestVelocity.withVelocity(Conversions.MPStoRPS(vel, shooterConstants.wheelCircumferenceMeters, 1)))
    }

    public void zerovelocity(){
        this.leftShooterSetPointMPS = 0;
        leftMotor.setControl(leftShootRequestVelocity.withVelocity(0));
    }

    public void requestMMVelocity(double velocityMPS){
        this.leftShooterSetPointMPS = velocityMPS;
        leftMotor.setControl(leftShootRequestMMVelocity.withVelocity(Conversions.MPStoRPS(velocityMPS,shooterConstants.wheelCircumferenceMeters,1)));
    }

    public void requestVoltage(double volts){
        leftMotor.setControl(shootRequestVoltage.withOutput(volts));
    }

    public void updateInputs(ShooterIOInputs inputs){
        BaseStatusSignal.refreshAll(leftShooterCurrent,rightShooterCurrent,leftShooterTemp,rightShooterTemp,leftShooterSpeedRPS,rightShooterSpeedRPS,leftVoltage,rightVoltage);
        //goes to each statussignal, updates
        inputs.appliedVolts = shootRequestVoltage.Output;
        inputs.appliedVelocity = leftShootRequestVelocity.Velocity;
        inputs.currentAmps = new double[] {
            leftShooterCurrent.getValueAsDouble(),
            rightShooterCurrent.getValueAsDouble()
        };
        inputs.temp = new double[] {
            leftShooterTemp.getValueAsDouble(),
            rightShooterTemp.getValueAsDouble()
        };
        inputs.shooterVelMPS = new double[]{
            Conversions.RPStoMPS(leftShooterSetPointMPS, shooterConstants.wheelCircumferenceMeters, 1),
            Conversions.RPStoMPS(rightShooterSpeedRPS, shooterConstants.wheelCircumferenceMeters, 1)
        };
        inputs.shooterSetpointMPS = leftShooterSetPointMPS;
        inputs.shooterVelRPS = new double[]{
            leftShooterSpeedRPS.getValueAsDouble(),
            rightShooterSpeedRPS.getValueAsDouble()
        };
        inputs.shooterVoltage = new double[]{
            leftVoltage.getValueAsDouble(),
            rightVoltage.getValueAsDouble()
        };

    }

}
