// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.Subsystems.EndEffector.EndEffectorIO;
import frc.robot.Subsystems.EndEffector.EndEffectorIOTalonFX;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.BeamBreak.BeamBreakIO;
import frc.robot.Subsystems.BeamBreak.BeamBreakIOAdafruit;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.commons.LoggedTunableNumber;
import frc.robot.Commands.TeleopSwerve;

public class RobotContainer {
    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1);
    
    private final Swerve swerve = new Swerve();

    private final ElevatorIO s_elevator = new ElevatorIOTalonFX();
    private final BeamBreakIO s_beambreak = new BeamBreakIOAdafruit(1, true); 
    private final IntakeIO s_intake = new IntakeIOTalonFX();
    private final EndEffectorIO s_endeffector = new EndEffectorIOTalonFX();
  
    private final Superstructure superstructure = new Superstructure(s_elevator, s_endeffector, s_intake, s_beambreak);

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
        

    }

    public Swerve getSwerve(){
        return swerve;
    }
    
    

}
