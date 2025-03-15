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
import frc.robot.Subsystems.Swerve.Swerve.FeedingStation;

public class FeedDriveAssist  extends Command{
    private final Swerve swerve;
    private final PIDController thetaController = new PIDController(5, 0, 0);
    private Rotation2d headingGoal;
    
    public FeedDriveAssist(Swerve swerve){
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        if(swerve.getFeed() == FeedingStation.LEFT ){
            headingGoal = new Rotation2d(-0.950546223291815 );
        }
        else{
            headingGoal = new Rotation2d(0.9097530329624376);
        }
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        double x = Math.pow(MathUtil.applyDeadband(RobotContainer.driver.getLeftY(), 0.1), 3);
        double y = Math.pow(MathUtil.applyDeadband(RobotContainer.driver.getLeftX(), 0.1),3);
        double dx;
        double dy;

        dx = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? x * -1 : x;
        dy =  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? y * -1: y;
       
        double thetaFeedback = thetaController.calculate(
            swerve.getPoseRaw().getRotation().getRadians(),
            headingGoal.getRadians()
        );
        thetaFeedback = MathUtil.clamp(thetaFeedback, -5, 5);

        swerve.requestDesiredState(dx * 4.72/2, dy * 4.72/2, thetaFeedback, true, false);

    }

    @Override
    public void end(boolean interrupted){

    }
}
