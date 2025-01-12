// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Amp.AmpIO;
import frc.robot.Subsystems.Amp.AmpIOTalonFX;
import frc.robot.Subsystems.Indexer.Indexer;
import frc.robot.Subsystems.Indexer.IndexerIO;
import frc.robot.Subsystems.Indexer.IndexerIOTalonFX;
import frc.robot.Subsystems.OTB_Intake.OTB_Intake;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIO;
import frc.robot.Subsystems.OTB_Intake.OTB_IntakeIOTalonFX;
import frc.robot.Subsystems.Shooter.ShooterArm;
import frc.robot.Subsystems.Shooter.ShooterArmIO;
import frc.robot.Subsystems.Shooter.ShooterArmIOTalonFX;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.AmpDriveAssistCommand;
import frc.robot.Subsystems.Swerve.PassAssistCommand;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autons.AutoConstants;
import frc.robot.autons.Autos;
import frc.commons.LoggedTunableNumber;
import frc.robot.Commands.TeleopSwerve;

public class RobotContainer {
  public static final XboxController controller = new XboxController(1);
  public static final CommandXboxController driver = new CommandXboxController(0);
    private final ShooterArmIO shooter = new ShooterArmIOTalonFX();
    private final OTB_IntakeIO otbIntake = new OTB_IntakeIOTalonFX();
    private final IndexerIO indexer = new IndexerIOTalonFX();
    private final AmpIO amp = new AmpIOTalonFX();
    private final Superstructure superstructure = new Superstructure(indexer, otbIntake, shooter,amp);
    private final Swerve swerve = new Swerve();

    public final EventLoop m_loop = new EventLoop();//¯\_(ツ)_/¯
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

    driver.y().onTrue(new InstantCommand(() -> swerve.zeroGyro()));

    driver.a().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.IDLE)));

    driver.leftBumper().whileTrue(new AmpDriveAssistCommand(swerve));

    driver.rightBumper().onTrue(new InstantCommand(() -> superstructure.requestPreShoot(AutoConstants.VelM, AutoConstants.RatioM, AutoConstants.DegM)));
    
    driver.rightTrigger().whileTrue(new PassAssistCommand(swerve, superstructure));
    
    driver.leftTrigger().onTrue(new InstantCommand(() -> superstructure.requestLowPass()));

    driver.b().onTrue(new InstantCommand(() -> superstructure.setState(SuperstructureStates.AUTO_INTAKE)));
    
    driver.x().onTrue(new InstantCommand(() -> superstructure.requestPreShoot(AutoConstants.VelM, AutoConstants.RatioM, AutoConstants.DegM)));
    
    new JoystickButton(controller, Button.kA.value).onTrue(
      new InstantCommand(() -> superstructure.setState(SuperstructureStates.IDLE))
      );
    new JoystickButton(controller, Button.kY.value).onTrue(
      new InstantCommand(() -> superstructure.requestPreShoot(AutoConstants.subwooferVelM, AutoConstants.subwooferRatioM, AutoConstants.subwooferDegM))
      );
    new JoystickButton(controller, Button.kRightBumper.value).onTrue(
      new InstantCommand(() -> superstructure.requestIntake())
    );
    new JoystickButton(controller, Button.kLeftBumper.value).onTrue(
      new InstantCommand(() -> superstructure.setState(SuperstructureStates.AMP_A))
    );
    new JoystickButton(controller, Button.kX.value).onTrue(
      new InstantCommand(() -> superstructure.setState(SuperstructureStates.AMP_B))
    );
    new JoystickButton(controller, Button.kB.value).onTrue(
      new InstantCommand(() -> superstructure.setState(SuperstructureStates.OUTTAKE))
    );
    
  }

  public Superstructure getSuperstructure(){
    return superstructure;
  }

  public Swerve getSwerve(){
    return swerve;
  }
}

