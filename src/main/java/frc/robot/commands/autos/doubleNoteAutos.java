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
import frc.robot.commands.Collect;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;

public class doubleNoteAutos {
    private static Translation2d midPose;
    private SwerveDrive swerveDrive;

    private final double bumperSize = 0;

    private final double startVelocity; //Velocities are in meters/second.
    private final double endVelocity;

    /** Example static factory for an autonomous command. */
    public doubleNoteAutos(SwerveDrive subsystem, double startVelocity, double endVelocity) {
        swerveDrive=subsystem;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
    }

    public Command DoubleNoteAuto1(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPoint1, startRotation);
        Translation2d endTranslation = UsefulPoints.Points.WingedNoteLeft;
        Rotation2d endRotation = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(endTranslation));
        Pose2d endPose = new Pose2d(endTranslation, endRotation);

//        midPose = new Translation2d(2,1);
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
//        internalPoints1.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 100);

        return Commands.sequence(swerveDrive.setInitPosition(startPose), /*new Shoot(),*/ trajCommand/*, new Collect(), new Shoot()*/);
    }

}
