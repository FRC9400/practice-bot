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
    public String selectedHeight = "L1";
    private double elevatorSetpoint = 0; 

    LoggedTunableNumber L3Height = new LoggedTunableNumber("Elevator/L3", Units.inchesToMeters(12.5));
    LoggedTunableNumber L2Height = new LoggedTunableNumber("Elevator/L2", Units.inchesToMeters(6.5));

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
        Logger.recordOutput("Selected Height", selectedHeight);
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

    public void requestMotionMagicCoral(){
        if (selectedHeight == "L1"){
            elevatorSetpoint = elevatorConstants.L1;
        }
        else if (selectedHeight == "L2"){
            elevatorSetpoint = L2Height.get();//elevatorConstants.L2;
        }
        else if (selectedHeight == "L3"){
            elevatorSetpoint = L3Height.get();//elevatorConstants.L3;
        }
        else if (selectedHeight == "L4"){
            elevatorSetpoint = elevatorConstants.L4;
        } else {
            return;
        }
        elevatorIO.resetMotionMagicConfigs(false);
        setState(ElevatorStates.SETPOINT);
    }

    public void requestMotionMagicAlgae(){
        if (selectedHeight == "L2"){
            elevatorSetpoint = elevatorConstants.L2Algae;
        } else if (selectedHeight == "L3"){
            elevatorSetpoint = elevatorConstants.L3Algae;
        } 
        elevatorIO.resetMotionMagicConfigs(false);
        setState(ElevatorStates.SETPOINT);
    }

    public void requestHold(){
        setState(ElevatorStates.SETPOINT);
    }

    public void requestElevatorDown(){
        elevatorSetpoint = Units.inchesToMeters(4);
        elevatorIO.resetMotionMagicConfigs(false);
        setState(ElevatorStates.SETPOINT);
    }

    public void requestSlow(){
        elevatorSetpoint = Units.inchesToMeters(1);
        elevatorIO.resetMotionMagicConfigs(true);
        setState(ElevatorStates.SETPOINT);
    }

    public void requestIdle(){
        setState(ElevatorStates.IDLE);
    }

    public void zeroSensor(){
        setState(ElevatorStates.ZERO_SENSOR);
    }

    public void setHeight(String height){
        selectedHeight = height;
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
