package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    public ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO elevatorIO){
        this.elevatorIO = elevatorIO;

    }

    @Override
    public void periodic(){
        elevatorIO.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void requestMotionMagic(double meters){
        elevatorIO.requestMotionMagic(meters);
    }

    public void requestVoltage(double volts){
        elevatorIO.requestVoltage(volts);
    }

    public void zeroSensor(){
        elevatorIO.zeroSensor();
    }

    public void coast(){
        elevatorIO.coast();
    }
}           
