package frc.robot.autons;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    public Command tune(String name){
        final AutoRoutine routine = autoFactory.newRoutine(name);
        final AutoTrajectory trajectory = routine.trajectory(name);
        routine.active().whileTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
        return routine.cmd();
    }

    public Command timedMid(){
        return Commands.sequence(new RunCommand(() -> swerve.requestDesiredState(5,0,0,false,false)).deadlineWith(new WaitCommand(2)), new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, false, false)));
    }

    public Command Preload(String pos, String traj){
        final AutoRoutine routine = autoFactory.newRoutine(pos + " " + traj);
        final AutoTrajectory trajectory = routine.trajectory(traj);
        routine.active().whileTrue(Commands.sequence(
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

    public Command Preload2(String pos, String traj){
        final AutoRoutine routine = autoFactory.newRoutine(pos + " " + traj);
        final AutoTrajectory trajectory = routine.trajectory(traj);
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

    public Command TwoPiece(String pos, String traj1, String traj2, String traj3){
        final AutoRoutine routine = autoFactory.newRoutine(pos + " " + traj1 + " " + traj2 + " " + traj3);
        final AutoTrajectory trajectory1 = routine.trajectory(traj1);
        final AutoTrajectory trajectory2 = routine.trajectory(traj2);
        final AutoTrajectory trajectory3 = routine.trajectory(traj3);
        routine.active().whileTrue(Commands.sequence(
            trajectory1.resetOdometry(),
            Commands.runOnce(() -> superstructure.setL4()),
            trajectory1.cmd(),
            Commands.runOnce(() -> superstructure.requestScore())
                .alongWith(Commands.waitUntil(() -> {
                    return superstructure.getState() == SuperstructureStates.IDLE;
                })),
            trajectory2.cmd()
                .alongWith(Commands.runOnce(() -> superstructure.requestIntake())),
            Commands.waitUntil(() -> {
                    return superstructure.getState() == SuperstructureStates.POST_INTAKE;
                }),
            trajectory3.cmd()
                .alongWith(Commands.runOnce(() -> superstructure.setL4())),
            Commands.runOnce(() -> superstructure.requestScore())
                .alongWith(Commands.waitUntil(() -> {
                    return superstructure.getState() == SuperstructureStates.IDLE;
                }))

        ));
        return routine.cmd();
    }

    public Command none(){
        return new InstantCommand();
    }
}