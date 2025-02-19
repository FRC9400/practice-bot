package frc.robot.Subsystems.Dealgae;

import org.littletonrobotics.junction.Logger;

public class Dealgae {
    private final DealgaeIO dealgaeIO;
    private DealgaeIOInputsAutoLogged inputs = new DealgaeIOInputsAutoLogged();
    private DealgaeStates dealgaeState = DealgaeStates.IDLE;
    private double voltageSetpoint = 0;

    public enum DealgaeStates{
        IDLE,
        DEALGAE,
        PROCESSOR
    }

    public Dealgae(DealgaeIO dealgaeIO){
        this.dealgaeIO = dealgaeIO;
    }

    public void Loop(){
        dealgaeIO.updateInputs(inputs);
        Logger.processInputs("Dealgae", inputs);
        Logger.recordOutput("DealgaeState", this.dealgaeState);
        switch(dealgaeState){
            case IDLE:
                dealgaeIO.requestVoltage(0);
                break;
            case DEALGAE:
                dealgaeIO.requestVoltage(voltageSetpoint);
                break;
            case PROCESSOR:
                dealgaeIO.requestVoltage(voltageSetpoint);
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(DealgaeStates.IDLE);
    }

    public void requestDealgae(double volts){
        voltageSetpoint = volts;
        setState(DealgaeStates.DEALGAE);
    }

    public void requestProcessor(double volts){
        voltageSetpoint = volts;
        setState(DealgaeStates.PROCESSOR);
    }

    public double getDealgaeCurrent(){
        return inputs.currentAmps;
    }

    public void setState(DealgaeStates nextState){
        this.dealgaeState = nextState;
    }

    public DealgaeStates getDealgaeState(){
        return this.dealgaeState;
    }

}