package frc.robot.Subsystems.EndEffector;

import org.littletonrobotics.junction.Logger;

public class EndEffector {
    private final EndEffectorIO endEffectorIO;
    private EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();
    private EndEffectorStates endEffectorState = EndEffectorStates.IDLE;
    private double voltageSetpoint = 0;

    public enum EndEffectorStates{
        IDLE,
        INTAKE,
        SCORE,
        HOLD_ALGAE
    }

    public EndEffector(EndEffectorIO endEffectorIO){
        this.endEffectorIO = endEffectorIO;
    }

    public void Loop(){
        endEffectorIO.updateInputs(inputs);
        Logger.processInputs("End Effector", inputs);
        Logger.recordOutput("EndEffectorState", this.endEffectorState);
        switch(endEffectorState){
            case IDLE:
                endEffectorIO.requestVoltage(0);
                break;
            case INTAKE:
                endEffectorIO.requestVoltage(voltageSetpoint);
                break;
            case SCORE:
                endEffectorIO.requestVoltage(voltageSetpoint);
            case HOLD_ALGAE:
                endEffectorIO.requestVoltage(voltageSetpoint);
            default:
                break;
        }
    }
    
    public void requestIdle(){
        setState(EndEffectorStates.IDLE);
    }

    public void requestIntake(double volts){
        voltageSetpoint = volts;
        setState(EndEffectorStates.INTAKE);
    }

    public void requestScore(double volts){
        voltageSetpoint = volts;
        setState(EndEffectorStates.SCORE);
    }

    public void requestHoldAlgae(double volts){
        voltageSetpoint = volts;
        setState(EndEffectorStates.HOLD_ALGAE);
    }

    public double getEndEffectorCurrent(){
        return inputs.currentAmps;
    }

    public void setState(EndEffectorStates nextState){
        this.endEffectorState = nextState;
    }

    public EndEffectorStates getEndEffectorState(){
        return this.endEffectorState;
    }
}