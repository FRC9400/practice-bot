package frc.robot.Subsystems.Swerve;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;

public class halfSpeed  extends Command{
    private final Swerve swerve;
    
    public halfSpeed(Swerve swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        double x = Math.pow(MathUtil.applyDeadband(RobotContainer.driver.getLeftY(), 0.1), 3);
        double y = Math.pow(MathUtil.applyDeadband(RobotContainer.driver.getLeftX(), 0.1),3);
        double theta = Math.pow(MathUtil.applyDeadband(RobotContainer.driver.getRightX(), 0.1),3);
        double dx;
        double dy;
        double dtheta;

        dx = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? x * -1 : x;
        dy =  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? y * -1: y;
        dtheta =  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? theta: theta * -1;

        swerve.requestDesiredState(dx * 4.72/3, dy * 4.72/3, dtheta * 4, true, false);

    }

    @Override
    public void end(boolean interrupted){

    }
}
