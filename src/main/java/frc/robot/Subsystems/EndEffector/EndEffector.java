package frc.robot.Subsystems.EndEffector;

import org.littletonrobotics.junction.Logger;

public class EndEffector {
    private final EndEffectorIO endEffectorIO;
    private EndEffectorInputsAutoLogged inputs = new EndEffectorInputsAutoLogged();
    private EndEffectorStates endEffectorState = EndEffectorStates.IDLE;
    private double[] voltageSetpoint = {0,1};

    public enum EndEffectorStates{
        IDLE,
        INTAKE,
        SCORE
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
                endEffectorIO.requestVoltage(0,0);
                break;
            case INTAKE:
                endEffectorIO.requestVoltage(voltageSetpoint[0],voltageSetpoint[1]);
                break;
            case SCORE:
                endEffectorIO.requestVoltage(voltageSetpoint[0],voltageSetpoint[1]);
            default:
                break;
        }
    }
    
    public void requestIdle(){
        setState(EndEffectorStates.IDLE);
    }

    public void requestIntake(double volts){
        voltageSetpoint[0] = volts;
        setState(EndEffectorStates.INTAKE);
    }

    public void requestScore(double volts){
        voltageSetpoint[0] = volts;
        setState(EndEffectorStates.SCORE);
    }

    public double getEndEffectorCurrent(){
        return inputs.currentAmps[0];
    }

    public void setState(EndEffectorStates nextState){
        this.endEffectorState = nextState;
    }

    public EndEffectorStates getEndEffectorState(){
        return this.endEffectorState;
    }
}