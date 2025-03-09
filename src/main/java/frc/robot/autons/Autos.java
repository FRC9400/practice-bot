package frc.robot.autons;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
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
            swerve::getEstimatedPose,
            swerve::resetOdometry,
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

    public Command Preload(String pos, String traj, String level){
        final AutoRoutine routine = autoFactory.newRoutine(pos + " " + traj);
        final AutoTrajectory trajectory = routine.trajectory(traj);
        Runnable setL;
        switch (level) {
            case "L1":
                setL = () -> superstructure.setL1();
                break;
            case "L2":
                setL = () -> superstructure.setL2();
                break;
            case "L3":
                setL = () -> superstructure.setL3();
                break;
            case "L4":
                setL = () -> superstructure.setL4();
                break;
            default:
                throw new IllegalArgumentException("Kill Yourself " + level);
        }
        routine.active().whileTrue(Commands.sequence(
            trajectory.resetOdometry(),
            Commands.runOnce(setL),
            trajectory.cmd(),
            Commands.runOnce(() -> superstructure.requestScore())
                .alongWith(Commands.waitUntil(() -> {
                    return superstructure.getState() == SuperstructureStates.IDLE;
                })).raceWith(Commands.run(() -> swerve.requestDesiredState(0, 0, 0, false, false)))
        ));
        return routine.cmd();
    }

    public Command TwoPiece(String pos, String traj1, String traj2, String traj3, String levelA, String levelB){
        final AutoRoutine routine = autoFactory.newRoutine(pos + " " + traj1 + " " + traj2 + " " + traj3);
        final AutoTrajectory trajectory1 = routine.trajectory(traj1);
        final AutoTrajectory trajectory2 = routine.trajectory(traj2);
        final AutoTrajectory trajectory3 = routine.trajectory(traj3);
        Runnable AsetL;
        Runnable BsetL;
        switch (levelA) {
            case "L1":
                AsetL = () -> superstructure.setL1();
                break;
            case "L2":
                AsetL = () -> superstructure.setL2();
                break;
            case "L3":
                AsetL = () -> superstructure.setL3();
                break;
            case "L4":
                AsetL = () -> superstructure.setL4();
                break;
            default:
                throw new IllegalArgumentException("Kill Yourself " + levelA);
        }
        switch (levelB) {
            case "L1":
                BsetL = () -> superstructure.setL1();
                break;
            case "L2":
                BsetL = () -> superstructure.setL2();
                break;
            case "L3":
                BsetL = () -> superstructure.setL3();
                break;
            case "L4":
                BsetL = () -> superstructure.setL4();
                break;
            default:
                throw new IllegalArgumentException("Kill Yourself " + levelB);
        }
        routine.active().whileTrue(Commands.sequence(
            trajectory1.resetOdometry(),
            Commands.runOnce(AsetL),
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
                .alongWith(Commands.runOnce(BsetL)),
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