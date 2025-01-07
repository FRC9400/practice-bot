package frc.robot.autons;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class Autos {

    public static Command idleCommand(Swerve swerve, Superstructure superstructure){
        return Commands.runOnce(() -> superstructure.requestIdle());
    }

    public static Command finishGyro(Swerve swerve, String startingPos) {
        if (startingPos.equals("mid")){
            return Commands.runOnce(() -> swerve. setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 180:0));
        }
        else if (startingPos.equals("source")) {
            return Commands.runOnce(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? -120 : 120));
        }
        else if (startingPos.equals("amp")){
            return Commands.runOnce(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 120 : -120));
        }

        return Commands.none();
    }
    

    public static Command requestMidShoot(Superstructure superstructure){
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> superstructure.requestAutoPreShoot(AutoConstants.VelM, AutoConstants.RatioM, AutoConstants.DegM))
            .andThen(new WaitUntilCommand(bool));
    }

    public static Command requestMidSubwooferShoot(Superstructure superstructure){
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> superstructure.requestAutoPreShoot(AutoConstants.subwooferVelM, AutoConstants.subwooferRatioM, AutoConstants.subwooferDegM))
            .andThen(new WaitUntilCommand(bool));

    }

    public static Command requestAmpShoot(Superstructure superstructure) {
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoPreShoot(AutoConstants.VelR, AutoConstants.RatioR, AutoConstants.DegR);
            } else {
                superstructure.requestAutoPreShoot(AutoConstants.VelL, AutoConstants.RatioL, AutoConstants.DegL);
            }
        })
            .andThen(new WaitUntilCommand(bool));
    }

    public static Command requestSourceShoot(Superstructure superstructure) {
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoPreShoot(AutoConstants.VelL, AutoConstants.RatioL, AutoConstants.DegL);
            } else {
                superstructure.requestAutoPreShoot(AutoConstants.VelR, AutoConstants.RatioR, AutoConstants.DegR);
            }
        })
            .andThen(new WaitUntilCommand(bool));
    }

    public static Command requestAmpSubwooferShoot(Superstructure superstructure) {
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoPreShoot(AutoConstants.subwooferVelR, AutoConstants.subwooferRatioR, AutoConstants.subwooferDegR);
            } else {
                superstructure.requestAutoPreShoot(AutoConstants.subwooferVelL, AutoConstants.subwooferRatioL, AutoConstants.subwooferDegL);
            }
        })
            .andThen(new WaitUntilCommand(bool));
    }

    public static Command requestSourceSubwooferShoot(Superstructure superstructure) {
        BooleanSupplier bool = () -> {
            return superstructure.getState() == SuperstructureStates.POST_SHOOT_AUTO;
         };
        return Commands.runOnce(() -> {
            if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
                superstructure.requestAutoPreShoot(AutoConstants.subwooferVelL, AutoConstants.subwooferRatioL, AutoConstants.subwooferDegL);
            } else {
                superstructure.requestAutoPreShoot(AutoConstants.subwooferVelR, AutoConstants.subwooferRatioR, AutoConstants.subwooferDegR);
            }
        })
            .andThen(new WaitUntilCommand(bool));
    }
    }