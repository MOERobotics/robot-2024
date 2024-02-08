// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.UsefulPoints;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;

public final class Autos {
private static Translation2d midPose;
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }
  public static Command doubleNoteAuto1(SwerveDrive swerveDrive/*, HeadSubsystem head, Arm arm*/) {
    SmartDashboard.putString("Started", "started");
    double bumperSize = 0;

    Rotation2d startRotation1 = new Rotation2d(0);
    //x = dist center of robot when robot is pushed against the wall.

    Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPoint1.getX(), UsefulPoints.Points.StartingPoint1.getY(),startRotation1);
    swerveDrive.resetOdometry(startPose1);

    Translation2d endPos = UsefulPoints.Points.WingedNoteLeft;
    Rotation2d endRotation1 = new Rotation2d(swerveDrive.getAngleBetweenSpeaker(endPos));
    Pose2d endPose1 = new Pose2d(endPos.getX(),endPos.getY(),endRotation1);

    double startVelocity1 = 0; //Velocities are in meters/second.
    double endVelocity1 = 0;
    //midPose = new Translation2d(2,1);
    ArrayList<Translation2d> internalPoints1 = new ArrayList<Translation2d>();
    //internalPoints1.add(midPose);
    Command command1 = swerveDrive.generateTrajectory(startPose1, endPose1, internalPoints1, startVelocity1, endVelocity1);

    //TODO:Add some commands for shooting once before leaving, once after trajectory.
    return Commands.sequence(/*shootCommand1,*/ command1 /*,shootCommand2*/);
  }
  public static Command doubleNoteAuto2(SwerveDrive swerveDrive/*, HeadSubsystem head, Arm arm*/) {
    SmartDashboard.putString("Started", "started");
    double bumperSize = 0;

    Rotation2d startRotation1 = new Rotation2d(0);
    //x = dist center of robot when robot is pushed against the wall.

    Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPoint2.getX(), UsefulPoints.Points.StartingPoint2.getY(),startRotation1);
    swerveDrive.resetOdometry(startPose1);

    Translation2d endPos = UsefulPoints.Points.WingedNoteMid;
    Rotation2d endRotation1 = new Rotation2d(swerveDrive.getAngleBetweenSpeaker(endPos));
    Pose2d endPose1 = new Pose2d(endPos.getX(),endPos.getY(),endRotation1);

    double startVelocity1 = 0; //Velocities are in meters/second.
    double endVelocity1 = 0;
    //midPose = new Translation2d(2,1);
    ArrayList<Translation2d> internalPoints1 = new ArrayList<Translation2d>();
    //internalPoints1.add(midPose);
    Command command1 = swerveDrive.generateTrajectory(startPose1, endPose1, internalPoints1, startVelocity1, endVelocity1);

    //TODO:Add some commands for shooting once before leaving, once after trajectory.
    return Commands.sequence(/*shootCommand1,*/ command1 /*,shootCommand2*/);
  }

  public static Command doubleNoteAuto3(SwerveDrive swerveDrive/*, HeadSubsystem head, Arm arm*/) {
    SmartDashboard.putString("Started", "started");
    double bumperSize = 0;

    Rotation2d startRotation1 = new Rotation2d(0);
    //x = dist center of robot when robot is pushed against the wall.

    Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPoint3.getX(), UsefulPoints.Points.StartingPoint3.getY(),startRotation1);
    swerveDrive.resetOdometry(startPose1);

    Translation2d endPos = UsefulPoints.Points.WingedNoteRight;
    Rotation2d endRotation1 = new Rotation2d(swerveDrive.getAngleBetweenSpeaker(endPos));
    Pose2d endPose1 = new Pose2d(endPos.getX(),endPos.getY(),endRotation1);

    double startVelocity1 = 0; //Velocities are in meters/second.
    double endVelocity1 = 0;
    //midPose = new Translation2d(2,1);
    ArrayList<Translation2d> internalPoints1 = new ArrayList<Translation2d>();
    //internalPoints1.add(midPose);
    Command command1 = swerveDrive.generateTrajectory(startPose1, endPose1, internalPoints1, startVelocity1, endVelocity1);

    //TODO:Add some commands for shooting once before leaving, once after trajectory.
    return Commands.sequence(/*shootCommand1,*/ command1 /*,shootCommand2*/);
  }

//  public static Command centerlineAuto1(SwerveDrive swerveDrive/*, HeadSubsystem head, Arm arm*/) {
//    SmartDashboard.putString("Started", "started");
//    double bumperSize = 0;
//
//    Rotation2d startRotation1 = new Rotation2d(0);
//    //x = dist center of robot when robot is pushed against the wall.
//
//    Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPoint3.getX(), UsefulPoints.Points.StartingPoint3.getY(),startRotation1);
//    swerveDrive.resetOdometry(startPose1);
//
//    Translation2d endPos = UsefulPoints.Points.CenterLineFarLeft;
//    Rotation2d endRotation1 = new Rotation2d(swerveDrive.getAngleBetweenSpeaker(endPos));
//    Pose2d endPose1 = new Pose2d(endPos.getX(),endPos.getY(),endRotation1);
//
//    Translation2d endPos2 = UsefulPoints.Points.WingedNoteRight;
//    Rotation2d endRotation2 = new Rotation2d(swerveDrive.getAngleBetweenSpeaker(endPos));
//    Pose2d endPose2 = new Pose2d(endPos.getX(),endPos.getY(),endRotation1);
//
//    double startVelocity1 = 0; //Velocities are in meters/second.
//    double endVelocity1 = 0;
//    double startVelocity2 = 0;
//    double endVelocity2 = 0;
//
//
//    //midPose = new Translation2d(2,1);
//    ArrayList<Translation2d> internalPoints1 = new ArrayList<Translation2d>();
//    ArrayList<Translation2d> internalPoints2 = new ArrayList<Translation2d>();
//    //internalPoints1.add(midPose);
//
//    //Command shoot1 = new shootCommand();
//    Command goCenterLine = swerveDrive.generateTrajectory(startPose1, endPose1, internalPoints1, startVelocity1, endVelocity1);
//    //Command pickUp = new Command();
//    Command goWing = swerveDrive.generateTrajectory(endPose1, endPose2, internalPoints2, startVelocity2, endVelocity2);
//    //Command shoot2 = new shootCommand();
//
//    //TODO:Add some commands for shooting once before leaving, once after trajectory.
//    return Commands.sequence(/*shoot1*/ goCenterLine, goWing /*,pickUp, shoot2*/);
//  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
