package frc.robot.autons;


import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;


public class Autos {
    private final Swerve swerve;
    private final Superstructure superstructure;
    private final AutoFactory autoFactory;


    public Autos(Swerve swerve, Superstructure superstructure){
        this.swerve = swerve;
    this.superstructure = superstructure;
    autoFactory = new AutoFactory(
            swerve::getPoseRaw,
            swerve::resetPose,
            swerve::followChoreoTraj,
            true,
            swerve);    
        }
    public AutoFactory getFactory() {
            return autoFactory;
        }
   
    public Command ProcessorToE(){
        final AutoRoutine routine = autoFactory.newRoutine("Preload from Processor");
        final AutoTrajectory trajectory = routine.trajectory("ProcessorToE");
        final AutoTrajectory eToCoral = routine.trajectory("EtoProcessorStation");

        routine.active().whileTrue(Commands.sequence(
            new InstantCommand(() -> swerve.setGyroStartingPosition(trajectory.getInitialPose().isPresent() ? trajectory.getInitialPose().get().getRotation().getDegrees() : 0)),
            trajectory.resetOdometry(),      
            Commands.runOnce(() -> superstructure.setL4()),
            trajectory.cmd(),
            Commands.runOnce(() -> superstructure.requestScore())
                .alongWith(Commands.waitUntil(() -> {
                    return superstructure.getState() == SuperstructureStates.IDLE;
                })).raceWith(Commands.run(() -> swerve.requestDesiredState(0, 0, 0, false, false))),
            eToCoral.cmd()
        ));
        return routine.cmd();
    }

    public Command PreloadandDealgaeMid(){
        final AutoRoutine routine = autoFactory.newRoutine("Preload and Dealgae from Mid");
        final AutoTrajectory trajectory = routine.trajectory("MidtoG");
       
        routine.active().whileTrue(Commands.sequence(
            new InstantCommand(() -> swerve.setGyroStartingPosition(trajectory.getInitialPose().isPresent() ? trajectory.getInitialPose().get().getRotation().getDegrees() : 0)),
            trajectory.resetOdometry(),      
            Commands.runOnce(() -> superstructure.setL4()),
            trajectory.cmd(),
            Commands.runOnce(() -> superstructure.requestScore())
                .alongWith(Commands.waitUntil(() -> {
                    return superstructure.getState() == SuperstructureStates.IDLE;
                })).raceWith(Commands.run(() -> swerve.requestDesiredState(0, 0, 0, false, false))),
                routine.trajectory("GAlgae").cmd().alongWith(Commands.runOnce(() -> superstructure.setL2()),
                Commands.runOnce(() -> superstructure.requestDealgae())))
        );
        return routine.cmd();
    }


    public Command PreloadMid(){
        final AutoRoutine routine = autoFactory.newRoutine("Preload from Mid");
        final AutoTrajectory trajectory = routine.trajectory("MidtoG");
        routine.active().whileTrue(Commands.sequence(
            trajectory.resetOdometry(),
            Commands.runOnce(() -> superstructure.setL4()),
            trajectory.cmd(),
            Commands.runOnce(() -> superstructure.requestScore())
                .alongWith(Commands.waitUntil(() -> {
                    return superstructure.getState() == SuperstructureStates.IDLE;
                })).raceWith(Commands.run(() -> swerve.requestDesiredState(0, 0, 0, false, false))))
        );
        return routine.cmd();
    }


    public Command PreloadCage(){
        final AutoRoutine routine = autoFactory.newRoutine("Preload from Cage");
        final AutoTrajectory trajectory = routine.trajectory("CagetoI");
        routine.active().whileTrue(Commands.sequence(
            trajectory.resetOdometry(),
            Commands.runOnce(() -> superstructure.setL4()),
            trajectory.cmd(),
            Commands.runOnce(() -> superstructure.requestScore())
                .alongWith(Commands.waitUntil(() -> {
                    return superstructure.getState() == SuperstructureStates.IDLE;
                })).raceWith(Commands.run(() -> swerve.requestDesiredState(0, 0, 0, false, false))))
        );
        return routine.cmd();
    }


    public Command Leave(){
        final AutoRoutine routine = autoFactory.newRoutine("Leave");
        final AutoTrajectory trajectory = routine.trajectory("MidtoG");
        routine.active().whileTrue(Commands.sequence(trajectory.resetOdometry()/*, trajectory.cmd(), Commands.runOnce(() -> superstructure.setL4())*/));
        return routine.cmd();
    }


    public Command none(){
        return new InstantCommand();
    }
}

