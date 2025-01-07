// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autons.Autos;
import frc.robot.autons.AutonomousSelector.modes;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  Command do_nothing;

  Command preload_mid;
  Command preload_amp;
  Command preload_source;
  Command four_note_mid;
  Command four_note_source;
  Command four_note_amp;
  Command two_note_mid;
  Command two_note_amp;
  Command two_note_source;

  private boolean built = false;

  @Override
  public void robotInit() {
    SignalLogger.setPath("/media/sda1/");
    
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.*/
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (DriverStation.getAlliance().isPresent() && !built){
      do_nothing = new InstantCommand();
      built = true;
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    if(m_robotContainer.getAutoCommand() == modes.DO_NOTHING){
      m_autonomousCommand = do_nothing;
    }

    if(m_robotContainer.getAutoCommand() == modes.PRELOAD_MID){
      m_autonomousCommand = preload_mid;
    }
    if(m_robotContainer.getAutoCommand() == modes.PRELOAD_AMP){
      m_autonomousCommand = preload_amp;
    }
    if(m_robotContainer.getAutoCommand() == modes.PRELOAD_SOURCE){
      m_autonomousCommand = preload_source;
    }
    if(m_robotContainer.getAutoCommand() == modes.FOUR_NOTE_MID){
      m_autonomousCommand = four_note_mid;
    }
    if(m_robotContainer.getAutoCommand() == modes.FOUR_NOTE_AMP){
      m_autonomousCommand = four_note_amp;
    }
    if(m_robotContainer.getAutoCommand() == modes.FOUR_NOTE_SOURCE){
      m_autonomousCommand = four_note_source;
    }
    if(m_robotContainer.getAutoCommand() == modes.TWO_NOTE_MID){
      m_autonomousCommand = two_note_mid;
    }
    if(m_robotContainer.getAutoCommand() == modes.TWO_NOTE_AMP){
      m_autonomousCommand = two_note_amp;
    }
    if(m_robotContainer.getAutoCommand() == modes.TWO_NOTE_SOURCE){
      m_autonomousCommand = two_note_source;
    }
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}