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
import frc.robot.Constants;
import frc.robot.UsefulPoints;
import frc.robot.commands.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Vision;

import java.util.ArrayList;
import java.util.Set;

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
    private Vision vision;

    /**
     * Example static factory for an autonomous command.
     */
    public doubleNoteAutos(SwerveDrive subsystem, Arm armSubsystem, ShooterSubsystem shooter, CollectorSubsystem collector, double startVelocity, double endVelocity, Vision vision) {
        this.vision=vision;
        swerveDrive = subsystem;
        this.armSubsystem = armSubsystem;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        this.shooter = shooter;
        this.collector = collector;
    } public doubleNoteAutos(SwerveDrive subsystem, Arm armSubsystem, ShooterSubsystem shooter, CollectorSubsystem collector, double startVelocity, double endVelocity) {
		swerveDrive = subsystem;
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
        Translation2d endTranslation = new Translation2d(UsefulPoints.Points.WingedNote2.getX()-Units.inchesToMeters(8),
                UsefulPoints.Points.WingedNote2.getY());
        Rotation2d endRotation = (swerveDrive.getAngleBetweenSpeaker(endTranslation));
        Pose2d endPose = new Pose2d(endTranslation, endRotation);

//        midPose = new Translation2d(endTranslation.getX()-Units.inchesToMeters(0),endTranslation.getY());
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
//        internalPoints.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command collectNote = new Collect(collector,.4,false);
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)),
                Commands.race(shootNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(()->swerveDrive.stopModules()), collectNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(112),Rotation2d.fromDegrees(-44)), Set.of(armSubsystem)),
//                Commands.race(headingCorrect.withTimeout(3), Commands.run(()->armSubsystem.holdPos(113.5, -42.19))),
//                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
		        //collect and shoot
        );
    }

    public Command DoubleNoteAuto1ScoreSub(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointC, startRotation);
        Translation2d endTranslation = new Translation2d(UsefulPoints.Points.WingedNote2.getX()-Units.inchesToMeters(8),
                UsefulPoints.Points.WingedNote2.getY());
        Rotation2d endRotation = (swerveDrive.getAngleBetweenSpeaker(endTranslation));
        Pose2d endPose = new Pose2d(endTranslation, endRotation);

//        midPose = new Translation2d(endTranslation.getX()-Units.inchesToMeters(0),endTranslation.getY());
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
//        internalPoints.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command collectNote = new Collect(collector,.4,false);
        Command trajCommand2 = swerveDrive.generateTrajectory(endPose,startPose,internalPoints,0,0);
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)),
                Commands.race(shootNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(()->swerveDrive.stopModules()), collectNote), Commands.run(()->armSubsystem.holdPos(81, -38))),
                Commands.runOnce(()->swerveDrive.stopModules()),
//                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(113.5), Rotation2d.fromDegrees(-42.19)), Set.of(armSubsystem)),
                Commands.race(trajCommand2.andThen(()-> swerveDrive.stopModules()), Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
//                Commands.race(headingCorrect.withTimeout(3), Commands.run(()->armSubsystem.holdPos(113.5, -42.19))),
//                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
                //collect and shoot
        );
    }

    public Command DoubleNoteAuto1ScoreSubWithObject(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointC, startRotation);
        Translation2d endTranslation = new Translation2d(UsefulPoints.Points.WingedNote2.getX()-Units.inchesToMeters(8),
                UsefulPoints.Points.WingedNote2.getY());
        Rotation2d endRotation = (swerveDrive.getAngleBetweenSpeaker(endTranslation));
        Pose2d endPose = new Pose2d(endTranslation, endRotation);

//        midPose = new Translation2d(endTranslation.getX()-Units.inchesToMeters(0),endTranslation.getY());
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
//        internalPoints.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command collectNote = new Collect(collector,.4,false);
        Command trajCommand2 = swerveDrive.generateTrajectory(endPose,startPose,internalPoints,0,0);
        Command objDetect = new AutoDriveToNoteCommand(swerveDrive,vision,1.0);
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-35)), Set.of(armSubsystem)),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(87), Rotation2d.fromDegrees(-56)), Set.of(armSubsystem)),
                Commands.race(shootNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(
                        Commands.parallel(
                                trajCommand.andThen(()->swerveDrive.stopModules()),
                                collectNote
                        ),
                        Commands.run(()->armSubsystem.holdPos(81, -38))),
                Commands.runOnce(()->swerveDrive.stopModules()),
//                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(113.5), Rotation2d.fromDegrees(-42.19)), Set.of(armSubsystem)),
                Commands.race(trajCommand2.andThen(()-> swerveDrive.stopModules()), Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
//                Commands.race(headingCorrect.withTimeout(3), Commands.run(()->armSubsystem.holdPos(113.5, -42.19))),
//                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
                //collect and shoot
        );
    }

    public Command DoubleNoteAuto2ScoreSub(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointB, UsefulPoints.Points.RotationOfStartingPointB);
        Translation2d endTranslation = new Translation2d(UsefulPoints.Points.WingedNote1.getX(),
                UsefulPoints.Points.WingedNote1.getY() + Units.inchesToMeters(6));
//        Translation2d endAnglePos = endTranslation.plus(new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(-10)));
        Rotation2d endRotation = (swerveDrive.getAngleBetweenSpeaker(endTranslation).plus(Rotation2d.fromDegrees(-5)));
        Pose2d endPose = new Pose2d(endTranslation,endRotation);
        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.StartingPointC,Rotation2d.fromDegrees(0));

//        midPose = new Translation2d(endTranslation.getX()-Units.inchesToMeters(0),endTranslation.getY());
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
//        internalPoints.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command collectNote = new Collect(collector,.4,false);
        Command trajCommand2 = swerveDrive.generateTrajectory(endPose,endPose2,internalPoints,0,0);
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-35)), Set.of(armSubsystem)),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)),
                Commands.race(shootNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(()->swerveDrive.stopModules()), collectNote), Commands.run(()->armSubsystem.holdPos(81, -38))),
                Commands.runOnce(()->swerveDrive.stopModules()),
//                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(113.5), Rotation2d.fromDegrees(-42.19)), Set.of(armSubsystem)),
                Commands.race(trajCommand2.andThen(()-> swerveDrive.stopModules()), Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
//                Commands.race(headingCorrect.withTimeout(3), Commands.run(()->armSubsystem.holdPos(113.5, -42.19))),
//                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
        );
    }

    public Command DoubleNoteAuto2(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d initPose = new Pose2d(UsefulPoints.Points.StartingPointB, UsefulPoints.Points.RotationOfStartingPointB);
        //Pose2d startPose = new Pose2d(initPose.getTranslation(),Rotation2d.fromRadians(swerveDrive.getAngleBetweenSpeaker(initPose.getTranslation())));
//        Translation2d endTranslation = UsefulPoints.Points.WingedNote1;
        Translation2d endTranslation = new Translation2d(UsefulPoints.Points.WingedNote1.getX(),
                UsefulPoints.Points.WingedNote1.getY() + Units.inchesToMeters(6));
//        Translation2d endAnglePos = endTranslation.plus(new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(-10)));
        Rotation2d endRotation = (swerveDrive.getAngleBetweenSpeaker(endTranslation).plus(Rotation2d.fromDegrees(-5)));
        Pose2d endPose = new Pose2d(endTranslation,endRotation);

        midPose = new Translation2d(Units.inchesToMeters(75), Units.inchesToMeters(260));
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        ArrayList<Translation2d> intP = new ArrayList<>();
        internalPoints.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(initPose,endPose,internalPoints, 0, 0);
        Pose2d finalPos = new Pose2d(UsefulPoints.Points.WingedNote1.getX()+Units.inchesToMeters(60),
                UsefulPoints.Points.WingedNote1.getY(),endRotation);
        Command crossLine = swerveDrive.generateTrajectory(endPose, finalPos, intP, 0,0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command collectNote = new Collect(collector,.4,false);
        Command headingCorrect = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()->AllianceFlip.apply(endRotation));
        return Commands.sequence(
                swerveDrive.setInitPosition(initPose),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-35)), Set.of(armSubsystem)),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)),
                Commands.race(shootNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(headingCorrect.withTimeout(1).andThen(()->swerveDrive.stopModules())), collectNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(112),
                        Rotation2d.fromDegrees(-37.5)), Set.of(armSubsystem)),
                Commands.race(Commands.waitSeconds(1), Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.parallel(crossLine.andThen(Commands.runOnce(()->swerveDrive.stopModules())),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
        );
    }

    public Command DMoveAuto(){
        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
        Pose2d endPose = new Pose2d(UsefulPoints.Points.CenterNote5.minus(new Translation2d(Units.inchesToMeters(54), Units.inchesToMeters(-18))),Rotation2d.fromDegrees(180));
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints,0,0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-35)), Set.of(armSubsystem)),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)),
                Commands.race(shootNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(()->swerveDrive.stopModules())), Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules())
        );
    }

    public Command DC5Auto(){
        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
        Pose2d endPose = new Pose2d(UsefulPoints.Points.CenterNote5.plus(new Translation2d(Units.inchesToMeters(-12), 0)),
                Rotation2d.fromDegrees(0));
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints,0,0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command collectNote = new Collect(collector,.4,false);
        Command headingCorrect = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()->AllianceFlip.apply(Rotation2d.fromDegrees(180)));
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-35)), Set.of(armSubsystem)),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)),
                Commands.race(shootNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(()->swerveDrive.stopModules()),collectNote), Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(headingCorrect.andThen(()-> swerveDrive.stopModules()),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
        );
    }

    public Command DC5C4PassC3(){
        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
        Pose2d endPose = new Pose2d(UsefulPoints.Points.CenterNote5,Rotation2d.fromDegrees(0));
        Pose2d startPose2 = new Pose2d(endPose.getTranslation(),Rotation2d.fromDegrees(90));
        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.CenterNote4,Rotation2d.fromDegrees(90));
        Pose2d startPose3 = new Pose2d(endPose2.getTranslation(),Rotation2d.fromDegrees(90));
        Pose2d endPose3 = new Pose2d(UsefulPoints.Points.CenterNote5,Rotation2d.fromDegrees(-25));
        Pose2d startPose4 = new Pose2d(endPose3.getTranslation(),Rotation2d.fromDegrees(90));
        Pose2d endPose4 = new Pose2d(UsefulPoints.Points.CenterNote3,Rotation2d.fromDegrees(90));
        Pose2d startPose5 = new Pose2d(endPose4.getTranslation(),Rotation2d.fromDegrees(90));
        Pose2d endPose5 = new Pose2d(UsefulPoints.Points.CenterNote5,Rotation2d.fromDegrees(-25));

        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints,0,0);
        Command trajCommand2 = swerveDrive.generateTrajectory(startPose2,endPose2,internalPoints,0,0);
        Command trajCommand3 = swerveDrive.generateTrajectory(startPose3,endPose3,internalPoints,0,0);
        Command trajCommand4 = swerveDrive.generateTrajectory(startPose4,endPose4,internalPoints,0,0);

        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command passNote = new NoteFeed(shooter,collector,()->1800);
        Command passAnotherNote = new NoteFeed(shooter,collector,()->1800);
        Command collectNote = new Collect(collector,.4,false);
        Command collectAnotherNote = new Collect(collector,.4,false);
        Command collectLastNote = new Collect(collector,.4,false);
        Command headingCorrect = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()->AllianceFlip.apply(Rotation2d.fromDegrees(-25)));
        Command headingCorrectCollect = new setHeading(swerveDrive,()->0.0,()->0.0,()->AllianceFlip.apply(Rotation2d.fromDegrees(90)));
        Command headingCorrect2 = new setHeading(swerveDrive,()->0.0,()->0.0,()->AllianceFlip.apply(Rotation2d.fromDegrees(-25)));
        Command headingCorrectCollect2 = new setHeading(swerveDrive,()->0.0,()->0.0,()->AllianceFlip.apply(Rotation2d.fromDegrees(90)));
        Command headingCorrect3 = new setHeading(swerveDrive,()->0.0,()->0.0,()->AllianceFlip.apply(Rotation2d.fromDegrees(180)));

        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)).andThen(Commands.waitSeconds(.15)),
                Commands.race(shootNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(()->swerveDrive.stopModules()), collectNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(4),
                Commands.runOnce(()->swerveDrive.stopModules()),//goes to point C5
                Commands.race(headingCorrect.andThen(()-> swerveDrive.stopModules()),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(passNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))), //heading correct and spit back
                Commands.race(headingCorrectCollect.andThen(()-> swerveDrive.stopModules()),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))), //swap back to 90
                Commands.race(Commands.parallel(trajCommand2.andThen(()->swerveDrive.stopModules()), collectAnotherNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(2),
                Commands.runOnce(()->swerveDrive.stopModules()), //goes to point C4
                Commands.race(Commands.parallel(trajCommand3.andThen(()->swerveDrive.stopModules())),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(2),
                Commands.runOnce(()->swerveDrive.stopModules()),Commands.race(headingCorrect2.andThen(()-> swerveDrive.stopModules()),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(passAnotherNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))), //spit out and then reorient to collect
                Commands.race(headingCorrectCollect2.andThen(()-> swerveDrive.stopModules()),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand4.andThen(()->swerveDrive.stopModules()), collectLastNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(2),
                Commands.runOnce(()->swerveDrive.stopModules()));
    }
    public Command DC5C4PassC3ObjDet(){
        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
        Pose2d endPose = new Pose2d(UsefulPoints.Points.CenterNote5,Rotation2d.fromDegrees(0));
        Pose2d startPose2 = new Pose2d(endPose.getTranslation(),Rotation2d.fromDegrees(90));
        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.CenterNote4,Rotation2d.fromDegrees(90));
        Pose2d startPose3 = new Pose2d(endPose2.getTranslation(),Rotation2d.fromDegrees(90));
        Pose2d endPose3 = new Pose2d(UsefulPoints.Points.CenterNote5,Rotation2d.fromDegrees(-25));
        Pose2d startPose4 = new Pose2d(endPose3.getTranslation(),Rotation2d.fromDegrees(90));
        Pose2d endPose4 = new Pose2d(UsefulPoints.Points.CenterNote3,Rotation2d.fromDegrees(90));
        Pose2d startPose5 = new Pose2d(endPose4.getTranslation(),Rotation2d.fromDegrees(90));
        Pose2d endPose5 = new Pose2d(UsefulPoints.Points.CenterNote5,Rotation2d.fromDegrees(-25));

        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints,0,0);
        Command trajCommand2 = swerveDrive.generateTrajectory(startPose2,endPose2,internalPoints,0,0);
        Command trajCommand3 = swerveDrive.generateTrajectory(startPose3,endPose3,internalPoints,0,0);
        Command trajCommand4 = swerveDrive.generateTrajectory(startPose4,endPose4,internalPoints,0,0);

        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command passNote = new NoteFeed(shooter,collector,()->1800);
        Command passAnotherNote = new NoteFeed(shooter,collector,()->1800);
        Command collectNote = new Collect(collector,.4,false);
        Command collectAnotherNote = new Collect(collector,.4,false);
        Command collectLastNote = new Collect(collector,.4,false);
        Command headingCorrect = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()->AllianceFlip.apply(Rotation2d.fromDegrees(-25)));
        Command headingCorrectCollect = new setHeading(swerveDrive,()->0.0,()->0.0,()->AllianceFlip.apply(Rotation2d.fromDegrees(90)));
        Command headingCorrect2 = new setHeading(swerveDrive,()->0.0,()->0.0,()->AllianceFlip.apply(Rotation2d.fromDegrees(-25)));
        Command headingCorrectCollect2 = new setHeading(swerveDrive,()->0.0,()->0.0,()->AllianceFlip.apply(Rotation2d.fromDegrees(90)));
        Command headingCorrect3 = new setHeading(swerveDrive,()->0.0,()->0.0,()->AllianceFlip.apply(Rotation2d.fromDegrees(180)));

        var driveToNote = new DriveToNoteCommand(
                swerveDrive,
                vision,
//                () -> Math.max(0, Math.hypot(driverJoystick.getRawAxis(0), driverJoystick.getRawAxis(1))-.05)*(maxMPS),
                () -> 1.0,
                () -> 0.0,
                () -> 0.0,
                (rumblePercent) -> {
                    SmartDashboard.putNumber("JoyRumble", rumblePercent);
                    //driverJoystick.setRumble(PS5Controller.RumbleType.kBothRumble, rumblePercent); //TODO: try different rumble types.
                },
                .56
        );
        var driveToNoteAgain = new DriveToNoteCommand(
                swerveDrive,
                vision,
//                () -> Math.max(0, Math.hypot(driverJoystick.getRawAxis(0), driverJoystick.getRawAxis(1))-.05)*(maxMPS),
                () -> 1.0,
                () -> 0.0,
                () -> 0.0,
                (rumblePercent) -> {
                    SmartDashboard.putNumber("JoyRumble", rumblePercent);
                    //driverJoystick.setRumble(PS5Controller.RumbleType.kBothRumble, rumblePercent); //TODO: try different rumble types.
                },
                .56
        );

        var driveToLastNote = new DriveToNoteCommand(
                swerveDrive,
                vision,
//                () -> Math.max(0, Math.hypot(driverJoystick.getRawAxis(0), driverJoystick.getRawAxis(1))-.05)*(maxMPS),
                () -> 1.0,
                () -> 0.0,
                () -> 0.0,
                (rumblePercent) -> {
                    SmartDashboard.putNumber("JoyRumble", rumblePercent);
                    //driverJoystick.setRumble(PS5Controller.RumbleType.kBothRumble, rumblePercent); //TODO: try different rumble types.
                },
                .56
        );


        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)).andThen(Commands.waitSeconds(.15)),
                Commands.race(shootNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(driveToNote.withTimeout(2))
                                .andThen(()->swerveDrive.stopModules()), collectNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(6),
                Commands.runOnce(()->swerveDrive.stopModules()),//goes to point C5
                Commands.race(headingCorrect.andThen(()-> swerveDrive.stopModules()),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(passNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))), //heading correct and spit back
                Commands.race(headingCorrectCollect.andThen(()-> swerveDrive.stopModules()),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))), //swap back to 90
                Commands.race(Commands.parallel(trajCommand2.andThen(driveToNoteAgain.withTimeout(2))
                                .andThen(()->swerveDrive.stopModules()), collectNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(6),
                Commands.runOnce(()->swerveDrive.stopModules()), //goes to point C4
                Commands.race(Commands.parallel(trajCommand3.andThen(()->swerveDrive.stopModules())),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(2),
                Commands.runOnce(()->swerveDrive.stopModules()),Commands.race(headingCorrect2.andThen(()-> swerveDrive.stopModules()),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(passAnotherNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))), //spit out and then reorient to collect
                Commands.race(headingCorrectCollect2.andThen(()-> swerveDrive.stopModules()),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand4.andThen(driveToLastNote.withTimeout(2))
                                .andThen(()->swerveDrive.stopModules()), collectNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(6),
                Commands.runOnce(()->swerveDrive.stopModules()));
    }
    public Command DoubleNoteAuto3(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointC, startRotation);
        Translation2d endTranslation = UsefulPoints.Points.WingedNote1;
        endTranslation = new Translation2d(endTranslation.getX()  - Units.inchesToMeters(15), endTranslation.getY() + Units.inchesToMeters(5));
        //Translation2d endAnglePos = endTranslation.plus(new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(-2)));
        Rotation2d endRotation = (swerveDrive.getAngleBetweenSpeaker(endTranslation));
        Pose2d endPose = new Pose2d(endTranslation,endRotation);

//        midPose = new Translation2d(endTranslation.getX()-Units.inchesToMeters(6),endTranslation.getY()-Units.inchesToMeters(10));
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
//        internalPoints.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command collectNote = new Collect(collector,.4,false);
        Command headingCorrect = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()->AllianceFlip.apply(endRotation));
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-35)), Set.of(armSubsystem)),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)),
                Commands.race(shootNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(headingCorrect.withTimeout(1).andThen(()->swerveDrive.stopModules())), collectNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(112/*armSubsystem.autoAim(()->swerveDrive.getEstimatedPose()).getX()*/),
                        Rotation2d.fromDegrees(-44.5/*armSubsystem.autoAim(()->swerveDrive.getEstimatedPose()).getY())*/)), Set.of(armSubsystem)),
//                Commands.race(headingCorrect.withTimeout(1), Commands.run(()->armSubsystem.holdPos(113.5, -42.19))),
//                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(Commands.run(()->swerveDrive.stopModules()).withTimeout(1), Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.parallel(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))

        );
    }

    public Command AMoveAuto(){//Roll straight past strating line
        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointA,Rotation2d.fromDegrees(0));
        Pose2d endPose = new Pose2d(new Translation2d(Units.inchesToMeters(150),UsefulPoints.Points.StartingPointA.getY()),Rotation2d.fromDegrees(0));
        ArrayList<Translation2d> internalPoints = new ArrayList<>();
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints,0,0);
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-35)), Set.of(armSubsystem)),
                Commands.parallel(trajCommand.andThen(()-> swerveDrive.stopModules()),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
        );
    }

    public Command DoubleNoteAuto4(){//TODO: Fix coordinates, create actual shoot and collect commands
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
        Translation2d endTranslation = new Translation2d(UsefulPoints.Points.WingedNote3.getX()-Units.inchesToMeters(19),UsefulPoints.Points.WingedNote3.getY()-Units.inchesToMeters(3));
        Rotation2d endRotation = (swerveDrive.getAngleBetweenSpeaker(endTranslation));
        SmartDashboard.putNumber("endRotationDub4", endRotation.getDegrees());
        Pose2d endPose = new Pose2d(endTranslation, Rotation2d.fromDegrees(0));
        Pose2d startPose2 = new Pose2d(endPose.getTranslation(),endRotation);
        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.DetourPointBottom,Rotation2d.fromDegrees(0));
        ArrayList<Translation2d> internalPoints2 = new ArrayList<Translation2d>();
        Translation2d midPose2 = endTranslation.minus(new Translation2d(Units.inchesToMeters(30),Units.inchesToMeters(60)));
        internalPoints2.add(midPose2);
        Command trajCommand2 = swerveDrive.generateTrajectory(startPose2,endPose2,internalPoints2,0,0);
        midPose = new Translation2d(Units.inchesToMeters(65),Units.inchesToMeters(175));
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        internalPoints.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command collectNote = new Collect(collector,.4,false);
        Command headingCorrect = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()->AllianceFlip.apply(endRotation));
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-35)), Set.of(armSubsystem)),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)),
                Commands.race(shootNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(headingCorrect.withTimeout(1).andThen(()->swerveDrive.stopModules())), collectNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(112),
                        Rotation2d.fromDegrees(-44.5)), Set.of(armSubsystem)),
                Commands.race(Commands.waitSeconds(1), Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
//                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.parallel(trajCommand2.andThen(()->swerveDrive.stopModules()),Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))

        );
    }

    public Command DoubleNoteAuto4ScoreSub(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
//        Translation2d endAnglePos = endTranslation.plus(new Translation2d(Units.inchesToMeters(7), Units.inchesToMeters(-10)));
        Translation2d endTranslation = new Translation2d(UsefulPoints.Points.WingedNote3.getX()-Units.inchesToMeters(22),UsefulPoints.Points.WingedNote3.getY()-Units.inchesToMeters(3));
        Pose2d endPose = new Pose2d(endTranslation, Rotation2d.fromDegrees(0));
        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.StartingPointC,Rotation2d.fromDegrees(0));

//        midPose = new Translation2d(endTranslation.getX()-Units.inchesToMeters(0),endTranslation.getY());
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
//        internalPoints.add(midPose);
        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints, 0, 0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command collectNote = new Collect(collector,.4,false);
        Command trajCommand2 = swerveDrive.generateTrajectory(endPose,endPose2,internalPoints,0,0);
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-35)), Set.of(armSubsystem)),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)),
                Commands.race(shootNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(()->swerveDrive.stopModules()), collectNote), Commands.run(()->armSubsystem.holdPos(81, -38))),
                Commands.runOnce(()->swerveDrive.stopModules()),
//                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(113.5), Rotation2d.fromDegrees(-42.19)), Set.of(armSubsystem)),
                Commands.race(trajCommand2.andThen(()-> swerveDrive.stopModules()), Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
//                Commands.race(headingCorrect.withTimeout(3), Commands.run(()->armSubsystem.holdPos(113.5, -42.19))),
//                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
        );
    }
    public Command CenterLineAuto1(){//TODO: Fix coordinates, create actual shoot and collect commands

        Rotation2d startRotation = new Rotation2d(0);
        //x = dist center of robot when robot is pushed against the wall.

        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointB, startRotation);
        Translation2d endTranslation1 = UsefulPoints.Points.WingedNote1;
        Rotation2d endRotation1 = (swerveDrive.getAngleBetweenSpeaker(endTranslation1));
        Pose2d endPose1 = new Pose2d(endTranslation1, endRotation1);

        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.WingedNote1, startRotation);
        Translation2d endTranslation2 = UsefulPoints.Points.CenterNote1;
        Rotation2d endRotation2 = (swerveDrive.getAngleBetweenSpeaker(endTranslation2));
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

    public Command rollOutAuto(){
        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
        Pose2d endPose = new Pose2d(UsefulPoints.Points.DetourPointBottom, new Rotation2d(0));
        var trajCommand = swerveDrive.generateTrajectory(startPose, endPose, new ArrayList<>(),0, 0);
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                trajCommand,
                Commands.runOnce(()->swerveDrive.stopModules())
        );
    }



}
