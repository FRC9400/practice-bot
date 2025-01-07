package frc.robot.Subsystems.OTB_Intake;

import org.littletonrobotics.junction.AutoLog;

public interface OTB_IntakeIO {
    @AutoLog
    public static class OTB_IntakeIOInputs{
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrent = 0.0;
        public double pivotSetpointDeg = 0.0;
        public double pivotSetpointRot = 0.0;
        public double pivotPosDeg = 0.0;
        public double pivotPosRot = 0.0;
        public double pivotTemperature = 0.0;
        public double pivotRPS = 0.0;

        public double intakeTemperature = 0.0;
        public double intakeAppliedVolts = 0.0;
        public double intakeCurrent = 0.0;
        public double intakeRPS = 0.0;
        public double intakeSetpointVolts = 0.0;
    }


public default void updateInputs(OTB_IntakeIOInputs inputs){}

public default void requestPivotVoltage(double voltage){}

public default void requestSetpoint(double angleDegrees){}

public default void requestIntakeVoltage(double voltage){}

public default void zeroPosition(){}

    
}