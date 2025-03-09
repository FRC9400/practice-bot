package frc.robot.Subsystems.Funnel;

import org.littletonrobotics.junction.Logger;

public class Funnel {
    private final FunnelIO funnelIO;
    private FunnelInputsAutoLogged inputs = new FunnelInputsAutoLogged();
    private FunnelStates funnelState = FunnelStates.IDLE;
    private double voltageSetpoint = 0;

    public enum FunnelStates{
        IDLE,
        INTAKE
    }

    public Funnel(FunnelIO funnelIO){
        this.funnelIO = funnelIO;
    }

    public void Loop(){
        funnelIO.updateInputs(inputs);
        Logger.processInputs("Funnel", inputs);
        Logger.recordOutput("Funnel", this.funnelState);
        switch(funnelState){
            case IDLE:
                funnelIO.requestVoltage(0);
                break;
            case INTAKE:
                funnelIO.requestVoltage(voltageSetpoint);
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(FunnelStates.IDLE);
    }

    public void requestIntake(double volts){
        voltageSetpoint = volts;
        setState(FunnelStates.INTAKE);
    }

    public void setState(FunnelStates nextState){
        this.funnelState = nextState;
    }
}
