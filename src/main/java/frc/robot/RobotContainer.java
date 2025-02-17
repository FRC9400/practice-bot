// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Dealgae.Dealgae;
import frc.robot.Subsystems.Dealgae.DealgaeIOTalonFX;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorIOTalonFX;
import frc.robot.Subsystems.LEDs.LEDs;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.commons.LoggedTunableNumber;
import frc.robot.Commands.TeleopSwerve;

public class RobotContainer {
    public static final CommandXboxController driver = new CommandXboxController(0);
    private final Elevator eleavtor = new Elevator(new ElevatorIOTalonFX());
    private final EndEffector endEffector = new EndEffector(new EndEffectorIOTalonFX());
    private final Dealgae dealgae = new Dealgae(new DealgaeIOTalonFX());
    private final Swerve swerve = new Swerve();
    private final LEDs leds = new LEDs();


    LoggedTunableNumber m3= new LoggedTunableNumber("Robot/meter", 0.05);

  
    public RobotContainer() {
    swerve.zeroWheels();
    swerve.zeroGyro();
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

    driver.x()
        .onTrue(swerve.driveSysIdCmd());

    driver.y()
        .onTrue(swerve.steerSysIdCmd());    

    driver.a()
        .onTrue(new RunCommand(() -> eleavtor.requestMotionMagic(m3.get())).until(() -> eleavtor.inputs.elevatorHeightMeters > m3.get() - 0.05)
        );
        
    

    driver.leftBumper()
        .whileTrue(new RunCommand(() -> endEffector.requestVoltage(0)));
      
    driver.rightBumper()
        .onTrue(new RunCommand(() -> leds.setColor()));

    driver.rightTrigger()
        .onTrue(new RunCommand(() -> leds.reset()));
      }


    public Swerve getSwerve(){
        return swerve;
    }


}

