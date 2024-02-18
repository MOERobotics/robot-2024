// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.UsefulPoints;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;

public class tripleNoteAutos {

    private SwerveDrive swerveDrive;

    private final double bumperSize = 0;

    private final double startVelocity; //Velocities are in meters/second.
    private final double endVelocity;

    /** Example static factory for an autonomous command. */
    public tripleNoteAutos(SwerveDrive subsystem, double startVelocity, double endVelocity) {
        swerveDrive=subsystem;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
    }

    public Command AW1W2(){//TODO: Fix coordinates, create actual shoot and collect commands
//go to W1 collect; go to B; shoot; W2 collect; go to D; shoot
        //traj 1
        Rotation2d startRotation1 = new Rotation2d(0);
        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointA, startRotation1);
        Translation2d wingedNote1 = UsefulPoints.Points.WingedNote1;
        Rotation2d endRotation1 = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(wingedNote1));
        Pose2d endPose1 = new Pose2d(UsefulPoints.Points.WingedNote1, endRotation1);
        //shoot beginning note; collect winged 1 note
        //start traj 2
        Rotation2d startRotation2 = new Rotation2d(swerveDrive.getYaw());
        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.WingedNote1, startRotation2);

        Rotation2d endRotation2 = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote2));
        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.WingedNote2, endRotation2);
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();


        Command traj1 = swerveDrive.generateTrajectory(startPose1,endPose1,internalPoints,0,0);
        Command traj2 = swerveDrive.generateTrajectory(startPose2,endPose2,internalPoints,0,0);

        return Commands.sequence(swerveDrive.setInitPosition(startPose1),traj1, traj2);
    }

    public Command EW3W2(){
        Rotation2d startRotation1 = new Rotation2d(0);
        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointE, startRotation1);
        Translation2d wingedNote3 = UsefulPoints.Points.WingedNote3;
        Rotation2d endRotation1 = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(wingedNote3));
        Pose2d endPose1 = new Pose2d(wingedNote3, endRotation1);

        Rotation2d startRotation2 = new Rotation2d(swerveDrive.getYaw());
        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.WingedNote3, startRotation2);

        Rotation2d endRotation2 = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote2));
        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.WingedNote2, endRotation2);
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();

        Command traj1 = swerveDrive.generateTrajectory(startPose1,endPose1,internalPoints,0,0);
        Command traj2 = swerveDrive.generateTrajectory(startPose2,endPose2,internalPoints,0,0);

        return Commands.sequence(traj1, traj2);
    }

    public Command BW1W2(){ //0 red collector; shoot blue
        //start at B, shoot at B, go to W1; collect W1; rotate robot to speaker; shoot note 1;
        //rotate to orientation w/ shooter head towards the winged note 2
        //collect W2; rotate to speaker; shoot W2
        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointB, UsefulPoints.Points.RotationOfStartingPointB);
        Rotation2d endRotation1 = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote1));
        Pose2d endPose1 = new Pose2d(UsefulPoints.Points.WingedNote1, endRotation1);

        Rotation2d startRotation2 = endRotation1;
        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.WingedNote1, startRotation2);
        Rotation2d endRotation2 = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote2));

        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.WingedNote2, endRotation2);



        //Translation2d mid = new Translation2d(60, 282.6);
        ArrayList<Translation2d> internalPoints1 = new ArrayList<Translation2d>();
        //internalPoints1.add(mid);
        ArrayList<Translation2d> internalPoints2 = new ArrayList<Translation2d>();


        Command traj1 = swerveDrive.generateTrajectory(startPose1, endPose1, internalPoints1,0,0);
        Command traj2 = swerveDrive.generateTrajectory(startPose2, endPose2, internalPoints2,0,0);

        return Commands.sequence(
                swerveDrive.setInitPosition(startPose1),
                //shoot
                traj1,
                //collect and shoot
                traj2
                //collect and shoot
        );
    }

    public Command DW3W2(){
        //start at D, shoot at D, go to W3; collect W3; rotate robot to speaker; shoot note 3;
        //rotate to orientation w/ shooter head towards the winged note 2
        //collect W2; rotate to speaker; shoot W2
        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
        Rotation2d endRotation1 = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote3));
        Pose2d endPose1 = new Pose2d(UsefulPoints.Points.WingedNote3, endRotation1);

        Rotation2d startRotation2 = new Rotation2d(swerveDrive.getYaw());
        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.WingedNote3, startRotation2);
        Rotation2d endRotation2 = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote2));

        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.WingedNote2, endRotation2);


        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();

        Command traj1 = swerveDrive.generateTrajectory(startPose1, endPose1, internalPoints,0,0);
        Command traj2 = swerveDrive.generateTrajectory(startPose2, endPose2, internalPoints,0,0);

        return Commands.sequence(
                //shoot
                traj1,
                //collect and shoot
                traj2
                //collect and shoot
        );
    }



}
