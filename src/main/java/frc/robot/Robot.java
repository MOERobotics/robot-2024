// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.pathfinding.LocalADStar;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autos.doubleNoteAutos;
import frc.robot.commands.autos.tripleNoteAutos;
import frc.robot.subsystems.SwerveDrive;

import java.util.Set;

public class Robot extends TimedRobot {
  private final Joystick driverJoystick = new Joystick(1); ///joystick imports
  private final Joystick functionJoystick = new Joystick(0);
  private final Joystick buttonBox = new Joystick(2);



  private Command m_autonomousCommand;
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();


  public static final boolean IS_LOGGING = true;
//
// private FortissiMOEContainer m_robotContainer; //= new FortissiMOEContainer();;
  private GenericContainer m_robotContainer;

  private void initRobotContainer(boolean force) {
    if (m_robotContainer != null)
      return;
    if (force || DriverStation.getAlliance().isPresent())
     m_robotContainer = SwerveBotContainer.getContainerSwerveBot();
     //  m_robotContainer = new FortissiMOEContainer();
  }

  @Override
  public void robotInit() {
      if(IS_LOGGING) {
          DataLogManager.start();
          DriverStation.startDataLog(DataLogManager.getLog());
      }
    initRobotContainer(false);


    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());

    m_chooser.setDefaultOption("Double Note Auto 1 (CW2)", new doubleNoteAutos(
        m_robotContainer.swerveSubsystem,
        m_robotContainer.armSubsystem,
        m_robotContainer.shooterSubsystem,
        m_robotContainer.collectorSubsystem,0,0).DoubleNoteAuto1());
    m_chooser.addOption("Double Note Auto 2 (BW1)", new doubleNoteAutos(m_robotContainer.swerveSubsystem, m_robotContainer.armSubsystem, m_robotContainer.shooterSubsystem, m_robotContainer.collectorSubsystem, 0,0).DoubleNoteAuto2());
    // m_chooser.addOption("Double Note Auto 3 (CW1)", new doubleNoteAutos(m_robotContainer.swerveSubsystem, m_robotContainer.armSubsystem, m_robotContainer.shooterSubsystem, m_robotContainer.collectorSubsystem, 0,0).DoubleNoteAuto3());
    m_chooser.addOption("Double Note Auto 4 (DW3)", new doubleNoteAutos(m_robotContainer.swerveSubsystem, m_robotContainer.armSubsystem, m_robotContainer.shooterSubsystem, m_robotContainer.collectorSubsystem, 0,0).DoubleNoteAuto4());
    // m_chooser.addOption("Triple Note Auto (BW1W2)", new tripleNoteAutos(m_robotContainer.swerveSubsystem, m_robotContainer.armSubsystem, m_robotContainer.shooterSubsystem, m_robotContainer.collectorSubsystem, 0,0).BW1W2());
    m_chooser.addOption("A Move Auto", new doubleNoteAutos(m_robotContainer.swerveSubsystem,m_robotContainer.armSubsystem,m_robotContainer.shooterSubsystem,m_robotContainer.collectorSubsystem,0,0).AMoveAuto());
    m_chooser.addOption("Triple Note CW1W2", new tripleNoteAutos(m_robotContainer.swerveSubsystem, m_robotContainer.armSubsystem, m_robotContainer.shooterSubsystem, m_robotContainer.collectorSubsystem, 0,0).CW1W2());
    m_chooser.addOption("Double Note Auto 1 Return Sub (CW2)", new doubleNoteAutos(m_robotContainer.swerveSubsystem, m_robotContainer.armSubsystem, m_robotContainer.shooterSubsystem, m_robotContainer.collectorSubsystem, 0,0).DoubleNoteAuto1ScoreSub());
    m_chooser.addOption("Double Note Auto 2 Return Sub (BW1)", new doubleNoteAutos(m_robotContainer.swerveSubsystem, m_robotContainer.armSubsystem, m_robotContainer.shooterSubsystem, m_robotContainer.collectorSubsystem, 0,0).DoubleNoteAuto2ScoreSub());
    m_chooser.addOption("Double Note Auto 4 Return Sub (DW3)", new doubleNoteAutos(m_robotContainer.swerveSubsystem, m_robotContainer.armSubsystem, m_robotContainer.shooterSubsystem, m_robotContainer.collectorSubsystem, 0,0).DoubleNoteAuto4ScoreSub());
    m_chooser.addOption("D Score Move", new doubleNoteAutos(m_robotContainer.swerveSubsystem,m_robotContainer.armSubsystem,m_robotContainer.shooterSubsystem,m_robotContainer.collectorSubsystem,0,0).DMoveAuto());
    m_chooser.addOption("D Score Collect (DC5)", new doubleNoteAutos(m_robotContainer.swerveSubsystem,m_robotContainer.armSubsystem,m_robotContainer.shooterSubsystem,m_robotContainer.collectorSubsystem,0,0).DC5Auto());
    m_chooser.addOption("4 Note Auto (CW2W1W3)", new tripleNoteAutos(m_robotContainer.swerveSubsystem,m_robotContainer.armSubsystem,m_robotContainer.shooterSubsystem,m_robotContainer.collectorSubsystem,0,0).CW1W2W3());
    m_chooser.addOption("centerLine Auto", new doubleNoteAutos(m_robotContainer.swerveSubsystem, m_robotContainer.armSubsystem, m_robotContainer.shooterSubsystem, m_robotContainer.collectorSubsystem, 0, 0).DC5C4PassC3());
    m_chooser.addOption("driveForward", new doubleNoteAutos(m_robotContainer.swerveSubsystem, m_robotContainer.armSubsystem, m_robotContainer.shooterSubsystem, m_robotContainer.collectorSubsystem, 0, 0).rollOutAuto());
    SmartDashboard.putData("chooser", m_chooser);
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putData("running command", CommandScheduler.getInstance());
    initRobotContainer(false);

    if (driverJoystick.getRawButton(1)) {
      m_robotContainer.gyroscope.reset();
      m_robotContainer.swerveSubsystem.setDesiredYaw(0);
    }


  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {


  }

  @Override
  public void autonomousInit() {
    initRobotContainer(true);
    // m_robotContainer.resetArmPos().schedule();
    m_autonomousCommand = m_chooser.getSelected();

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
  public void teleopPeriodic() {

    if(buttonBox.getRawButton(8)){
      m_robotContainer.armSubsystem.wristPowerController(.1);
    }

    if(buttonBox.getRawButton(6)){
      m_robotContainer.shooterSubsystem.setMaxShooterSpeeds(2300,2300);
    }else{
      m_robotContainer.shooterSubsystem.setMaxShooterSpeeds(3500,3500);
    }
    
    if(buttonBox.getRawButton(2)){
      m_robotContainer.armSubsystem.wristPowerController(-.1);
    }

    if(functionJoystick.getRawButton(7)){
      m_robotContainer.armSubsystem.shoulderPowerController(-.2);
    }

    if(driverJoystick.getRawButton(8)){
      m_robotContainer.armSubsystem.shoulderPowerController(.2);
    }

  }

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
