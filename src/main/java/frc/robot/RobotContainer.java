// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Dealgae.DealgaeIO;
import frc.robot.Subsystems.Dealgae.DealgaeIOTalonFX;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.EndEffector.EndEffectorIO;
import frc.robot.Subsystems.EndEffector.EndEffectorIOTalonFX;
import frc.robot.Subsystems.LEDs.LEDs;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Commands.TeleopSwerve;

public class RobotContainer {
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1);
    
    private final DealgaeIO s_dealgae = new DealgaeIOTalonFX();
    private final EndEffectorIO s_endeffector = new EndEffectorIOTalonFX();
    private final ElevatorIO s_elevator = new ElevatorIOTalonFX();
    private final LEDs s_leds = new LEDs();
    private final Superstructure superstructure = new Superstructure(s_dealgae, s_elevator, s_endeffector, s_leds);
    private final Swerve swerve = new Swerve();
  
    public RobotContainer() {
    swerve.zeroGyro();
    swerve.zeroWheels();
    swerve.setDefaultCommand(
        new TeleopSwerve(
            swerve, 
            () -> -driver.getRawAxis(XboxController.Axis.kLeftY.value),
            () -> -driver.getRawAxis(XboxController.Axis.kLeftX.value), 
            () -> -driver.getRawAxis(XboxController.Axis.kRightX.value)
          
        )
    );

    configureBindings();

    }

    private void configureBindings() {

        driver.start()
            .onTrue(new InstantCommand(() -> superstructure.requestIdle()));
        
        driver.back()
            .onTrue(new InstantCommand(() -> swerve.zeroGyro())); 
        
        driver.rightBumper()
            .onTrue(new InstantCommand(() -> superstructure.requestElevatorDown()));
        
        driver.rightTrigger()
            .onTrue(new InstantCommand(() -> superstructure.requestProcessor()));
        
        driver.leftBumper()
            .onTrue(new InstantCommand(() -> superstructure.requestScore()));
        
        driver.leftTrigger()
            .onTrue(new InstantCommand(() -> superstructure.requestDealgae()));

        operator.leftBumper()
            .onTrue(new InstantCommand(() -> superstructure.requestIntake()));
        
        operator.leftTrigger()
            .onTrue(new InstantCommand(() -> superstructure.requestIntake2()));

        operator.rightBumper()
            .onTrue(new InstantCommand(() -> superstructure.requestOuttake()));

        operator.rightTrigger()
            .onTrue(new InstantCommand(() -> superstructure.requestIdle()));

        operator.x()
            .onTrue(new InstantCommand(() -> superstructure.setL1()));

        operator.y()
            .onTrue(new InstantCommand(() -> superstructure.setL2()));
        
        operator.a()
            .onTrue(new InstantCommand(() -> superstructure.setL3()));

        operator.b()
            .onTrue(new InstantCommand(() -> superstructure.setL4()));

        
    }




    public Swerve getSwerve(){
        return swerve;
    }

    public Superstructure getSuperstructure(){
        return superstructure;
    }

}
