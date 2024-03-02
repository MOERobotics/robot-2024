// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AllianceFlip;
import frc.robot.UsefulPoints;
import frc.robot.commands.Collect;
import frc.robot.commands.setHeading;
import frc.robot.commands.shootSpeakerCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;

public class doubleNoteAutos {
    private static Translation2d midPose;
    private SwerveDrive swerveDrive;
    private Arm armSubsystem;

    private final double bumperSize = 0;

    private final double startVelocity; //Velocities are in meters/second.
    private final double endVelocity;
//    private Command collectorPosition = armSubsystem.goToPoint(Rotation2d.fromDegrees(105), Rotation2d.fromDegrees(-31));
//    private Command shootPosition = armSubsystem.goToPoint(Rotation2d.fromDegrees(104), Rotation2d.fromDegrees(-41));
    private ShooterSubsystem shooter;
    private CollectorSubsystem collector;

    /** Example static factory for an autonomous command. */
    public doubleNoteAutos(SwerveDrive subsystem, Arm armSubsystem, ShooterSubsystem shooter, CollectorSubsystem collector, double startVelocity, double endVelocity) {
        swerveDrive=subsystem;
        this.armSubsystem = armSubsystem;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        this.shooter = shooter;
        this.collector = collector;
    }

    public doubleNoteAutos(SwerveDrive subsystem, double startVelocity, double endVelocity) {
        swerveDrive=subsystem;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
    }

    public Command DoubleNoteAuto1(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointC, startRotation);
        Translation2d endTranslation = UsefulPoints.Points.WingedNote2;
        Rotation2d endRotation = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(endTranslation));
        Pose2d endPose = new Pose2d(endTranslation, endRotation);

//        midPose = new Translation2d(endTranslation.getX()-Units.inchesToMeters(0),endTranslation.getY());
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
//        internalPoints.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 0);

        return Commands.sequence(
				swerveDrive.setInitPosition(startPose),
		        trajCommand
		        //collect and shoot
        );
    }

    public Command DoubleNoteAuto2(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d initPose = new Pose2d(UsefulPoints.Points.StartingPointB, UsefulPoints.Points.RotationOfStartingPointB);
        //Pose2d startPose = new Pose2d(initPose.getTranslation(),Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(initPose.getTranslation())));
        Translation2d endTranslation = UsefulPoints.Points.WingedNote1;
        Rotation2d endRotation = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(endTranslation));
        Pose2d endPose = new Pose2d(endTranslation,endRotation);

        midPose = new Translation2d(Units.inchesToMeters(75), Units.inchesToMeters(250));
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        internalPoints.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(initPose,endPose,internalPoints, 0, 0);

        //Command aimSpeaker = new setHeading(swerveDrive, ()->0.0,()->0.0,Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(endPose.getTranslation())));
        //Command shootNote = new shootSpeakerCommand(shooter,collector);
        //Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        //Command collectNote = new Collect(collector,1,false);
        return Commands.sequence(
                swerveDrive.setInitPosition(initPose),
                //collectorPosition,
                //shootNote,
                Commands.parallel(trajCommand/*, collectNote*/)
//                collectorPosition,
                //shootAnotherNote

                //shootPosition
		        //shootNote
        );
    }

    public Command DoubleNoteAuto3(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointC, startRotation);
        Translation2d endTranslation = UsefulPoints.Points.WingedNote1;
        Rotation2d endRotation = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(endTranslation));
        Pose2d endPose = new Pose2d(endTranslation, endRotation);

//        midPose = new Translation2d(endTranslation.getX()-Units.inchesToMeters(6),endTranslation.getY()-Units.inchesToMeters(10));
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
//        internalPoints.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 0);

        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                trajCommand
        );
    }

    public Command DoubleNoteAuto4(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointD, startRotation);
        Translation2d endTranslation = UsefulPoints.Points.WingedNote3;
        Rotation2d endRotation = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(endTranslation));
        Pose2d endPose = new Pose2d(endTranslation, endRotation);

//        midPose = new Translation2d(2,1);
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
//        internalPoints1.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 0);

        return Commands.sequence(
				swerveDrive.setInitPosition(startPose),
		        //shoot
		        trajCommand
	            //collect and shoot
        );
    }

    public Command CenterLineAuto1(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointB, startRotation);
        Translation2d endTranslation1 = UsefulPoints.Points.WingedNote1;
        Rotation2d endRotation1 = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(endTranslation1));
        Pose2d endPose1 = new Pose2d(endTranslation1, endRotation1);

        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.WingedNote1, startRotation);
        Translation2d endTranslation2 = UsefulPoints.Points.CenterNote1;
        Rotation2d endRotation2 = Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(endTranslation2));
        Pose2d endPose2 = new Pose2d(endTranslation2, endRotation2);


//        midPose = new Translation2d(2,1);
        ArrayList<Translation2d> internalPoints1 = new ArrayList<Translation2d>();

        ArrayList<Translation2d> internalPoints2 = new ArrayList<Translation2d>();
//        internalPoints1.add(midPose);
        Command trajCommand1 = swerveDrive.generateTrajectory(startPose1,endPose1,internalPoints1, 0, 0);
        Command trajCommand2 = swerveDrive.generateTrajectory(startPose2, endPose2, internalPoints2, 0, 0);

        return Commands.sequence(
				swerveDrive.setInitPosition(startPose1)
		        //new Shoot(),
		        //trajCommand1,
		        //trajCommand2,
		        //collect and shoot
		);
    }

    public Command FCenterAuto(){//TODO: Create actual shoot and collect commands
        Rotation2d startRotation = new Rotation2d(0);
        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointF, startRotation);
        Translation2d endTranslation = UsefulPoints.Points.CenterNote5;
        Pose2d endPose = new Pose2d(endTranslation, startRotation);
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        internalPoints.add(UsefulPoints.Points.DetourPointBottom);
        Command trajCommand1 = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 0);

        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.CenterNote5,startRotation);
        Rotation2d endRotation2 = new Rotation2d().fromDegrees(120);
        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.StartingPointD,endRotation2);
        ArrayList<Translation2d> internalPoints2 = new ArrayList<Translation2d>();
        internalPoints2.add(UsefulPoints.Points.DetourPointBottom);
        Command trajCommand2 = swerveDrive.generateTrajectory(startPose2,endPose2,internalPoints2, 0, 0);

        return Commands.sequence(
				swerveDrive.setInitPosition(startPose),
		        /*new Shoot(),*/
		        trajCommand1,
		        trajCommand2
				//collect and shoot
        );
    }



}
