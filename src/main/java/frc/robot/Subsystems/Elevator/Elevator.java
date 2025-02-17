package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.SignalLogger;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.elevatorConstants;

public class Elevator extends SubsystemBase {
    private final ElevatorIO elevatorIO;
    public ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final SysIdRoutine elevatorRoutine;

    public Elevator(ElevatorIO elevatorIO){
        this.elevatorIO = elevatorIO;
        elevatorRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(4), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism((volts) -> elevatorIO.requestVoltage(volts.in(Volts)), null,
                        this));

    }

    public Command elevatorSysIdCmd(){
        return Commands.sequence(
            //this.runOnce(() -> SignalLogger.start()),
            this.runOnce(() -> elevatorIO.requestVoltage(1)),
            /*elevatorRoutine
                    .quasistatic(Direction.kForward)
                    .until(() -> inputs.elevatorHeightMeters > elevatorConstants.maxHeightMeters),*/ //Keep in mind the max height is around 0.6
            Commands.waitSeconds(1),
            this.runOnce(() -> elevatorIO.requestVoltage(0))/*,
            Commands.waitSeconds(1),
            elevatorRoutine
                    .quasistatic(Direction.kReverse)
                    .until(() -> inputs.elevatorHeightMeters < 0.05), //Keep in mind the max height is around 0.6
            this.runOnce(() -> elevatorIO.requestVoltage(0)),
            Commands.waitSeconds(1),

            elevatorRoutine
                    .dynamic(Direction.kForward)
                    .until(() -> inputs.elevatorHeightMeters > elevatorConstants.maxHeightMeters), //Keep in mind the max height is around 0.6
            this.runOnce(() -> elevatorIO.requestVoltage(0)),
            Commands.waitSeconds(1),

            elevatorRoutine
                    .dynamic(Direction.kReverse)
                    .until(() -> inputs.elevatorHeightMeters < 0.05), //Keep in mind the max height is around 0.6
            this.runOnce(() -> elevatorIO.requestVoltage(0)),
            Commands.waitSeconds(1),
            this.runOnce(() -> SignalLogger.stop())*/);
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
