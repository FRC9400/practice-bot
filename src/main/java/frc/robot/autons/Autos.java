package frc.robot.autons;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class Autos {
    private final Swerve s_Swerve;
    private final AutoFactory autoFactory;

    public Autos(Swerve s_Swerve){
        this.s_Swerve = s_Swerve;
        autoFactory = new AutoFactory(
            s_Swerve::getPoseRaw,
            s_Swerve::resetPose,
            s_Swerve::followChoreoTraj,
            true,
            s_Swerve);    
        }
    public AutoFactory getFactory() {
            return autoFactory;
        }
    
    public Command testChoreo(){
        final AutoRoutine routine = autoFactory.newRoutine("test");
        final AutoTrajectory trajectory = routine.trajectory("test");
        routine.active().whileTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
        return routine.cmd();
    }

    public Command none(){
        return Commands.none();
    }

    public Command resetOdometry(){
        final AutoRoutine routine = autoFactory.newRoutine("test reset");
        final AutoTrajectory trajectory = routine.trajectory("test");
        routine.active().whileTrue(trajectory.resetOdometry());
        return routine.cmd();
    }
}