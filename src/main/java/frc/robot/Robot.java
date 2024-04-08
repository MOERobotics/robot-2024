// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.RobotController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.pathfinding.LocalADStar;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDrive;

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
  private void initRobotContainer(boolean force) {
    if (m_robotContainer != null)
      return;
    if (force || DriverStation.getAlliance().isPresent())
      m_robotContainer = selectRobot();
  }

  @Override
  public void robotInit() {
    if (m_robotContainer == null)
      m_robotContainer = selectRobot();
//    m_robotContainer = new FortissiMOEContainer();
    initRobotContainer(false);
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData("running command", CommandScheduler.getInstance());
    initRobotContainer(false);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if (m_robotContainer!= null) m_robotContainer.resetArmPos();
  }

  @Override
  public void autonomousInit() {
    initRobotContainer(true);
    m_robotContainer.resetArmPos().schedule();
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
    initRobotContainer(true);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.resetArmPos().schedule();
    m_robotContainer.buttonsCommand.schedule();
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
