package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.SignalLogger;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Intake extends SubsystemBase{
    private final IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final SysIdRoutine pivotSysID;

    public Intake(IntakeIO intakeIO){
        this.intakeIO = intakeIO;
        pivotSysID = new SysIdRoutine(
                new SysIdRoutine.Config(null, Volts.of(3), null,
                        (state) -> SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism((volts) -> intakeIO.requestPivotVoltage(volts.in(Volts)), null,
                        this));
    }

    public Command runSysIdCmd() {
        return Commands.sequence(
                this.runOnce(() -> SignalLogger.start()),
                pivotSysID
                        .quasistatic(Direction.kForward)
                        .until(Math.abs(inputs.pivotPositionDeg) > 80),
                this.runOnce(() -> intakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                pivotSysID
                        .quasistatic(Direction.kReverse)
                        .until(inputs.pivotPositionDeg < 5),
                this.runOnce(() -> intakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kForward)
                        .until(Math.abs(inputs.pivotPositionDeg) > 110),
                this.runOnce(() -> intakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),

                pivotSysID
                        .dynamic(Direction.kReverse)
                        .until(inputs.pivotPositionDeg < 5),
                this.runOnce(() -> intakeIO.requestPivotVoltage(0)),
                Commands.waitSeconds(1),
                this.runOnce(() -> SignalLogger.stop()));
    }

    @Override
    public void periodic(){
        intakeIO.updateTunableNumbers();
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }


}
