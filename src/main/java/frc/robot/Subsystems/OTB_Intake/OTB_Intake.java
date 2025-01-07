
package frc.robot.Subsystems.OTB_Intake;

import org.littletonrobotics.junction.Logger;

public class OTB_Intake{
    private final OTB_IntakeIO otbIntakeIO;
    private OTB_IntakeIOInputsAutoLogged inputs = new OTB_IntakeIOInputsAutoLogged();
    private IntakeStates intakeState = IntakeStates.IDLE;

    public enum IntakeStates{
        IDLE,
        INTAKE,
        SETPOINT,
        OUTTAKE,
        DOWN
    }

    public OTB_Intake(OTB_IntakeIO otbIntakeIO) {
        this.otbIntakeIO = otbIntakeIO;
      }


    public void Loop() {
        otbIntakeIO.updateInputs(inputs);
        Logger.processInputs("OTB_Intake", inputs);
        Logger.recordOutput("OTB_IntakeState", this.intakeState);
        switch(intakeState){
            case IDLE:
                otbIntakeIO.requestPivotVoltage(0);
                otbIntakeIO.requestIntakeVoltage(0);
                break;
            case INTAKE:
                otbIntakeIO.requestIntakeVoltage(3);
                otbIntakeIO.requestSetpoint(39.48);
                break;
            case SETPOINT:
                otbIntakeIO.requestIntakeVoltage(0);
                otbIntakeIO.requestSetpoint(0);
                break;
            case OUTTAKE:
                otbIntakeIO.requestIntakeVoltage(-3);
                otbIntakeIO.requestSetpoint(39.48);
                break;
            case DOWN:
                otbIntakeIO.requestSetpoint(39.48);
                otbIntakeIO.requestIntakeVoltage(0);
            default:
                break;

        }
    }

    public void requestIdle(){
        setState(IntakeStates.IDLE);
    }
    
    public void requestIntake(){
        setState(IntakeStates.INTAKE);
    }

    public void requestSetpoint(){
        setState(IntakeStates.SETPOINT);
    }

    public void requestOuttake(){
        setState(IntakeStates.OUTTAKE);
    }

    public void requestDown(){
        setState(IntakeStates.DOWN);
    }

    public void setState(IntakeStates nextState){
        this.intakeState = nextState;
    }
      
    
    
    


}
