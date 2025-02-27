// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autons.AutonomousSelector;
import frc.robot.autons.Autos;
import frc.robot.autons.AutonomousSelector.modes;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Autos autos;
  
  Command tune_x;
  Command tune_y;
  Command tune_theta;
  Command pos1preload;
  Command pos2preload;
  Command pos3preload;
  Command pos1twopiece;
  Command pos2twopiece;
  Command pos3twopiece;

  private AutonomousSelector selector;

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
    selector = new AutonomousSelector(m_robotContainer.getSwerve(), autos);
    autos = new Autos(m_robotContainer.getSwerve(), m_robotContainer.getSuperstructure());
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
      tune_x = autos.tune("tuneX");
      tune_y = autos.tune("tuneY");
      tune_theta = autos.tune("tuneTheta");
      pos1preload = autos.Preload("Position 1", "Pos1toI", "L2");
      pos2preload = autos.Preload("Position 2", "Pos2toI", "L2");
      pos3preload = autos.Preload("Position 3", "Pos3toF", "L2");
      pos1twopiece = autos.TwoPiece("Position 1", "Pos1toI", "ItoSource", "SourcetoJ", "L2", "L2");
      pos2twopiece = autos.TwoPiece("Position 2", "Pos2toI", "ItoSource", "SourcetoJ", "L2", "L2");
      pos3twopiece = autos.TwoPiece("Position 3", "Pos3toF", "FtoSource", "SourcetoE", "L2", "L2");
      built = true;
    }

    }
  

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    
    if(selector.get() == modes.DO_NOTHING){
      m_autonomousCommand = new InstantCommand();
    }

    if(selector.get() == modes.TUNE_X){
      m_autonomousCommand = tune_x;
    }

    if(selector.get() == modes.TUNE_Y){
      m_autonomousCommand = tune_y;
    }

    if(selector.get() == modes.TUNE_THETA){
      m_autonomousCommand = tune_theta;
    }

    if(selector.get() == modes.POS1_PRELOAD_TO_I){
      m_autonomousCommand = pos1preload;
    }

    if(selector.get() == modes.POS2_PRELOAD_TO_I){
      m_autonomousCommand = pos2preload;
    }

    if(selector.get() == modes.POS3_PRELOAD_TO_F){
      m_autonomousCommand = pos3preload;
    }

    if(selector.get() == modes.POS1_TWOPIECE_I_J){
      m_autonomousCommand = pos1twopiece;
    }

    if(selector.get() == modes.POS2_TWOPIECE_I_J){
      m_autonomousCommand = pos2twopiece;
    }

    if(selector.get() == modes.POS3_TWOPIECE_F_E){
      m_autonomousCommand = pos3twopiece;
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