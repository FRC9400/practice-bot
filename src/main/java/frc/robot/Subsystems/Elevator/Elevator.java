package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import com.cter.phoenix6.SignalLogger;
import static edu.wpi.first.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysis.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysis.SysIdRoutine.Direction;
import frc.robot.Constants.elevatorConstants;


public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine elevatorRoutine;
    private ElevatorStates elevatorstate = ElevatorStates.idle;
    private String position = "L1";
    
    public enum elevatorStates{
        IDLE,
        SETPOINT,
        DOWN
    }

    public Elevator(ElevatorIO elevatorIO){
        this.elevatorIO = elevatorIO;
        elevatorRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null,
                (states) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> elevatorIO.requestVoltage(volts.in(Volts)), null,
            this));
    }

    public Command elevatorSysIdCmd(){
        return Commands.sequence(
            this.runOnce(() -> SignalLogger.start()),
            elevatorRoutine
                .quasistatic(Direction.kForward)
                .until(() -> inputs.elevatorHeightMeters > elevatorConstants.maxHeightMeters - 0.2),
            this.runOnce(() -> elevatorIO.requestVoltage(0)),
            Commands.waitSeconds(1),

            elevatorRoutine
                .quasistatic(Direction.kReverse)
                .until(() -> inputs.elevatorHeightMeters < 0.1),
            This.runOnce(() -> elevatorIO.requestVoltage(0)),
            Commands.waitSeconds(1),

            elevatorRoutine
                .dynamic(Direction.kForward)
                .until(() -> inputs.elevatorHeightMeters > elevatorConstants.maxHeightMeters - 0.2),
            this.runOnce(() -> elevatorIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            this.runOnce(() -> SignalLogger.stop()));
    }

    public requestMotionMagic(double meters) {
        elevatorIO.requestMotionMagic(meters);
    }

    public requestMotionMagic(double volts) {
        elevatorIO.requestMotionMagic(volts);
    }

    public void setState(ElevatorStates next) {elevatorStates = next;}

    public ElevatorStates getState(){return elevatorstate;}

    public String getSetpointString(){return position;}

    public void requestIdle(){
        elevatorState = ElevatorStates.IDLE;
    }

    public void requestL1(){
        elevatorState = ElevatorStates.SETPOINT;
        position = "L1";
    }

    public void request L2(){
        elevatorState = ElevatorStates.SETPOINT;
        position = "L2";
    }

    public void request L3(){
        elevatorState = ElevatorStates.SETPOINT;
        position = "L3";
    }

    public void request L4(){
        elevatorState = ElevatorStates.SETPOINT;
        position - "L4";
    }

    public void requestDown(){
        elevatorState = ElevatorStates.DOWN;
    }

    @OVERRIDE
    public void periodic(){
        elevatorIO.updateInputs(inputs);
        Logger.proccessInputs("Elevator", inputs);

        switch(elevatorState){
            case IDLE:
                elevatorIO.requestVoltage(0);
                break;
            case SETPOINT:
                if(position == "L4"){
                    elevatorIO.requestMotionMagic(elevatorConstants.L4);
                }
                else if(position == "L3"){
                    elevatorIO.requestMotionMagic(elevatorConstants.L3);
                }
                else if(position == "L2"){
                    elevatorIO.requestMotionMagic(elevatorConstants.L2);
                }
                else if(position == "L1"){
                    elevatorIO.requestMotionMagic(elevatorConstants.L1);
                }
                else{
                    elevatorIO.requestVoltage(0);
                }
                break;
            case Down:
                elevatorIO.requestMotionMagic(0);
                break:
        }
    }
}