package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import frc.commons.Conversions;
import frc.commons.LoggedTunableNumber;
import frc.robot.Constants.elevatorConstants;

public class Elevator {
    private final ElevatorIO elevatorIO;
    private ElevatorStates elevatorState = ElevatorStates.IDLE;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double elevatorSetpoint = 0; 

    public enum ElevatorStates{
        IDLE,
        SETPOINT, 
        ZERO_SENSOR
    }

    public Elevator(ElevatorIO elevatorIO){
        this.elevatorIO = elevatorIO;

    }

    public void Loop(){
        elevatorIO.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("ElevatorState", elevatorState);
        Logger.recordOutput("Elevator Setpoint", elevatorSetpoint);

        switch(elevatorState){
            case IDLE:
                elevatorIO.requestVoltage(0);
                break;
            case SETPOINT:
                elevatorIO.requestMotionMagic(elevatorSetpoint);
                break;
            case ZERO_SENSOR:
                elevatorIO.zeroSensor(0);
                break;
            default:
                break;
        }
    }

    public void requestL1(){
        elevatorSetpoint = elevatorConstants.L1;
        setState(ElevatorStates.SETPOINT);
    }

    public void requestL2(){
        elevatorSetpoint = elevatorConstants.L2;
        setState(ElevatorStates.SETPOINT);
    }

    public void requestL3(){
        elevatorSetpoint = elevatorConstants.L3;
        setState(ElevatorStates.SETPOINT);
    }

    public void requestL4(){
        elevatorSetpoint = elevatorConstants.L4;
        setState(ElevatorStates.SETPOINT);
    }

    public void requestHold(){
        setState(ElevatorStates.SETPOINT);
    }

    public void requestIdle(){
        setState(ElevatorStates.IDLE);
    }

    public void zeroSensor(){
        setState(ElevatorStates.ZERO_SENSOR);
    }

    public boolean atSetpoint(){
        return Math.abs(inputs.elevatorHeightMeters - elevatorSetpoint) < Units.inchesToMeters(0.3);
    }

    public void setState(ElevatorStates nextState){
        this.elevatorState = nextState;
    }

    public ElevatorStates getElevatorState(){
        return this.elevatorState;
    }
}           
