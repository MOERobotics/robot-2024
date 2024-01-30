// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }
  public static Command doubleNoteAuto(SwerveDrive subsystem) {
    Pose2d startPose1 = new Pose2d();
    Pose2d endPose1 = new Pose2d();
    double startVelocity1 = 0; //Velocities are in meters/second.
    double endVelocity1 = 0;
    ArrayList<Translation2d> internalPoints1 = new ArrayList<Translation2d>();
    Command command1 = new DriveTrajectory(subsystem, startPose1, endPose1, internalPoints1, startVelocity1, endVelocity1);
    //TODO:Add some commands for shooting once before leaving, once after trajectory.
    return Commands.sequence(/*shootCommand1,*/ command1 /*,shootCommand2*/);
  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
