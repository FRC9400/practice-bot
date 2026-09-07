package frc.robot.Subsystems.Pivot;

import org.littletonrobotics.junction.Logger;

import frc.commons.Conversions;
import frc.robot.Constants.pivotConstants;

public class Pivot {
    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    private PivotStates pivotState = PivotStates.IDLE;
    private double pivotSetpoint = pivotConstants.stowAngleDegrees;

    public enum PivotStates{
        IDLE,
        SETPOINT,
        ZERO_SENSOR
    }

    public Pivot(PivotIO pivotIO){
        this.pivotIO = pivotIO;
    }

    public void Loop(){
        pivotIO.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        Logger.recordOutput("Pivot", this.pivotState);
        Logger.recordOutput("Pivot", pivotSetpoint);

        switch(pivotState){
            case IDLE:
                pivotIO.requestVoltage(0);
                break;
            case SETPOINT:
                pivotIO.requestMotionMagic(pivotSetpoint);
                break;
            case ZERO_SENSOR:
                pivotIO.zeroSensor(Conversions.DegreesToRotations(pivotConstants.stowAngleDegrees, pivotConstants.gearRatio));
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(PivotStates.IDLE);
    }

    public void requestStow(){
        pivotSetpoint = pivotConstants.stowAngleDegrees;
        setState(PivotStates.SETPOINT);
    }

    public void requestSubwoofer(){
        pivotSetpoint = pivotConstants.subwooferAngleDegrees;
        setState(PivotStates.SETPOINT);
    }

    public void requestPodium(){
        pivotSetpoint = pivotConstants.podiumAngleDegrees;
        setState(PivotStates.SETPOINT);
    }

    public void requestAngle(double degrees){
        pivotSetpoint = degrees;
        setState(PivotStates.SETPOINT);
    }

    public void zeroSensor(){
        setState(PivotStates.ZERO_SENSOR);
    }

    public boolean atSetpoint(){
        return Math.abs(inputs.pivotAngleDegrees - pivotSetpoint) < pivotConstants.toleranceDegrees;
    }

    public void setState(PivotStates nextState){
        this.pivotState = nextState;
    }

    public PivotStates getPivotState(){
        return this.pivotState;
    }

}
