package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.Logger;

public class Intake {
    private final IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private IntakeStates intakeState = IntakeStates.IDLE;
    private double voltageSetpoint = 0;

    public enum IntakeStates{
        IDLE,
        INTAKE
    }

    public Intake(IntakeIO intakeIO){
        this.intakeIO = intakeIO;
    }

    public void Loop(){
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake", this.intakeState);
        switch(intakeState){
            case IDLE:
                intakeIO.requestVoltage(0);
                break;
            case INTAKE:
                intakeIO.requestVoltage(voltageSetpoint);
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(IntakeStates.IDLE);
    }

    public void requestIntake(double volts){
        voltageSetpoint = volts;
        setState(IntakeStates.INTAKE);
    }

    public double getIntakeCurrent(){
        return inputs.currentAmps;
    }

    public void setState(IntakeStates nextState){
        this.intakeState = nextState;
    }
}