// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.pathfinding.LocalADStar;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDrive;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static final boolean IS_LOGGING = true;
//
// private FortissiMOEContainer m_robotContainer; //= new FortissiMOEContainer();;
  private SwerveBotContainer m_robotContainer;

  private void initRobotContainer(boolean force) {
    if (m_robotContainer != null)
      return;
    if (force || DriverStation.getAlliance().isPresent())
     m_robotContainer = new SwerveBotContainer();
     //  m_robotContainer = new FortissiMOEContainer();
  }

  @Override
  public void robotInit() {
      if(IS_LOGGING) {
          DataLogManager.start();
          DriverStation.startDataLog(DataLogManager.getLog());
      }
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
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    initRobotContainer(true);
    // m_robotContainer.resetArmPos().schedule();
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
   // m_robotContainer.resetArmPos().schedule();
   // m_robotContainer.buttonsCommand.schedule();
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
