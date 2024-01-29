// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.pathplanner.lib.pathfinding.LocalADStar;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  /**
   * Set this to true before a competition weekend
   */
  public static final boolean IS_COMPETITION = false;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  private RobotContainer selectRobot() {
    // Competition override
    if (IS_COMPETITION)
      return new FortissiMOEContainer();
    // Try to read from USB
    try {
      Path robotFile = Paths.get("/u/robot.txt").toRealPath();
      var robotName = Files.readString(robotFile);
      var rc = RobotContainer.forName(robotName);
      if (rc.isPresent())
        return rc.get();
    } catch (IOException | NullPointerException ex) {
      // ignored
    }
    // Try to load from RoboRIO firmware
    var rc = RobotContainer.forName(RobotController.getComments());
    // By default, we select Fortissimoe
    return rc.orElseGet(FortissiMOEContainer::new);
  }

  @Override
  public void robotInit() {
    if (m_robotContainer == null)
      m_robotContainer = selectRobot();
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }


  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
