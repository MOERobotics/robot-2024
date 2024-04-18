// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS5Controller;
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

import javax.xml.crypto.dsig.TransformService;
import java.util.ArrayList;
import java.util.Set;

@SuppressWarnings("UnnecessaryLocalVariable")
public class tripleNoteAutos {

    private SwerveDrive swerveDrive;

    private final double bumperSize = 0;

    private final double startVelocity; //Velocities are in meters/second.
    private final double endVelocity;
    private ShooterSubsystem shooter;
    private CollectorSubsystem collector;
    private Arm armSubsystem;
    private final Vision vision;

    /** Example static factory for an autonomous command. */
    public tripleNoteAutos(SwerveDrive subsystem, Arm armSubsystem, ShooterSubsystem shooter, CollectorSubsystem collector, double startVelocity, double endVelocity) {
        swerveDrive=subsystem;
        this.armSubsystem = armSubsystem;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        this.shooter = shooter;
        this.collector = collector;
        this.vision = new Vision();
    }

    public tripleNoteAutos(SwerveDrive subsystem, double startVelocity, double endVelocity) {
        swerveDrive=subsystem;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        this.vision = new Vision();
    }
    public Command CW1W2(){//TODO: Fix coordinates, create actual shoot and collect commands
        //go to W1 collect; go to B; shoot; W2 collect; go to D; shoot
        //traj 1
        Rotation2d startRotation = new Rotation2d(0);
        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointC, startRotation);
        Rotation2d endRotation = (swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote2));
        Translation2d endTranslation = new Translation2d(UsefulPoints.Points.WingedNote2.getX()-Units.inchesToMeters(8),
                UsefulPoints.Points.WingedNote2.getY());
        Pose2d endPose = new Pose2d(endTranslation, endRotation); //goes from start c to point w2

        Translation2d endTranslation2 = UsefulPoints.Points.StartingPointC.plus(
                new Translation2d(Units.inchesToMeters(6), 0));
//        Rotation2d startRotation2 = new Rotation2d(swerveDrive.getYaw());
        Pose2d startPose2 = new Pose2d(endTranslation, endRotation);
        Rotation2d endRotation2 = Rotation2d.fromDegrees(0);
        Pose2d endPose2 = new Pose2d(endTranslation2, endRotation2); //goes from point w2 to start c

        Translation2d endTranslation3 = new Translation2d(UsefulPoints.Points.WingedNote1.getX(),
                UsefulPoints.Points.WingedNote1.getY() + Units.inchesToMeters(6));
        Rotation2d startRotation3 = endRotation2;
        Pose2d startPose3 = new Pose2d(endTranslation2, startRotation3);
        Rotation2d endRotation3 = (swerveDrive.getAngleBetweenSpeaker(endTranslation3).plus(Rotation2d.fromDegrees(-5)));
        Pose2d endPose3 = new Pose2d(endTranslation3,endRotation3); //startC to W1

        Translation2d endTranslation4 = endTranslation2;
        Rotation2d startRotation4 = endRotation3;
        Pose2d startPose4 = new Pose2d(endPose3.getTranslation(), startRotation4);
        Rotation2d endRotation4= endRotation2;
        Pose2d endPose4 = new Pose2d(endTranslation4, endRotation4); //W1 to start C

        Translation2d endTranslation5 = new Translation2d(UsefulPoints.Points.WingedNote3.getX()-Units.inchesToMeters(19),UsefulPoints.Points.WingedNote3.getY()-Units.inchesToMeters(3));
        Rotation2d endRotation5 = (swerveDrive.getAngleBetweenSpeaker(endTranslation));
        Rotation2d startRotation5 = endRotation4;
        Pose2d startPose5 = new Pose2d(endPose4.getTranslation(), startRotation5);
        Pose2d endPose5 = new Pose2d(endTranslation5, endRotation5);// start C to W3

        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        ArrayList<Translation2d> internalPoints2 = new ArrayList<>();
        ArrayList<Translation2d> internalPoints3 = new ArrayList<>();
        ArrayList<Translation2d> internalPoints4 = new ArrayList<>();
        ArrayList<Translation2d> internalPoints5 = new ArrayList<>();


        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints,0,0);
        Command trajCommand2 = swerveDrive.generateTrajectory(startPose2,endPose2,internalPoints2,0,0);
        Command trajCommand3 = swerveDrive.generateTrajectory(startPose3,endPose3,internalPoints3,0,0);
        Command trajCommand4 = swerveDrive.generateTrajectory(startPose4,endPose4,internalPoints4,0,0);
        Command trajCommand5 = swerveDrive.generateTrajectory(startPose5,endPose5,internalPoints5,0,0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command shootLastNote = new shootSpeakerCommand(shooter, collector);

        Command collectNote = new Collect(collector,0.4,false);
        Command collectNote2 = new Collect(collector,0.4,false);

        Command headingCorrect = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()-> AllianceFlip.apply(endRotation));
        Command headingCorrect2 = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()-> AllianceFlip.apply(endRotation3));
        Command headingCorrect4 = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()-> AllianceFlip.apply(endRotation4));
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)).andThen(Commands.waitSeconds(.15)),
                Commands.race(shootNote,Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(()->swerveDrive.stopModules()), collectNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(trajCommand2.andThen(()->swerveDrive.stopModules()),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand3.andThen(()->swerveDrive.stopModules()), collectNote2),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(Commands.parallel(trajCommand4.andThen(headingCorrect4.withTimeout(.25)).andThen(()->swerveDrive.stopModules())),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.parallel(shootLastNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
        );
    }

    public Command CW1W2W3(){
        //go to W1 collect; go to B; shoot; W2 collect; go to D; shoot
        //traj 1
        Rotation2d startRotation = new Rotation2d(0);
        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointC, startRotation);
        Rotation2d endRotation = (swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote2));
        Translation2d endTranslation = new Translation2d(UsefulPoints.Points.WingedNote2.getX()-Units.inchesToMeters(10),
                UsefulPoints.Points.WingedNote2.getY());
        Pose2d endPose = new Pose2d(endTranslation, endRotation); //goes from start c to point w2

        Translation2d endTranslation2 = UsefulPoints.Points.StartingPointC.plus(
                new Translation2d(Units.inchesToMeters(7), 0));
//        Rotation2d startRotation2 = new Rotation2d(swerveDrive.getYaw());
        Pose2d startPose2 = new Pose2d(endTranslation, endRotation);
        Rotation2d endRotation2 = Rotation2d.fromDegrees(0);
        Pose2d endPose2 = new Pose2d(endTranslation2, endRotation2); //goes from point w2 to start c

        Translation2d endTranslation3 = new Translation2d(UsefulPoints.Points.WingedNote3.getX()-Units.inchesToMeters(10),
                UsefulPoints.Points.WingedNote3.getY());
        Rotation2d startRotation3 = endRotation2;
        Pose2d startPose3 = new Pose2d(endTranslation2, startRotation3);
        Rotation2d endRotation3 = Rotation2d.fromDegrees(0);
        Pose2d endPose3 = new Pose2d(endTranslation3,endRotation3); //startC to W3

        Translation2d endTranslation4 = endTranslation2;
        Rotation2d startRotation4 = endRotation3;
        Pose2d startPose4 = new Pose2d(endPose3.getTranslation(), startRotation4);
        Rotation2d endRotation4= endRotation2;
        Pose2d endPose4 = new Pose2d(endTranslation4, endRotation4); //W3 to start C

        Translation2d endTranslation5 = new Translation2d(UsefulPoints.Points.WingedNote1.getX(),UsefulPoints.Points.WingedNote1.getY() + Units.inchesToMeters(6));
        Rotation2d endRotation5 = (swerveDrive.getAngleBetweenSpeaker(endTranslation));
        Rotation2d startRotation5 = endRotation4;
        Pose2d startPose5 = new Pose2d(endPose4.getTranslation(), startRotation5);
        Pose2d endPose5 = new Pose2d(endTranslation5, endRotation5);// start C to W1

        Translation2d endTranslation6 = endTranslation4;
        Rotation2d endRotation6 = (swerveDrive.getAngleBetweenSpeaker(endTranslation6));
        Rotation2d startRotation6 = endRotation5;
        Pose2d startPose6 = new Pose2d(endPose5.getTranslation(), startRotation6);
        Pose2d endPose6 = new Pose2d(endTranslation6, endRotation6);// W1 to start C

        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        ArrayList<Translation2d> internalPoints2 = new ArrayList<>();
        ArrayList<Translation2d> internalPoints3 = new ArrayList<>();
        internalPoints3.add(endTranslation3.plus(new Translation2d(Units.inchesToMeters(-24),Units.inchesToMeters(5))));
        ArrayList<Translation2d> internalPoints4 = new ArrayList<>();
        ArrayList<Translation2d> internalPoints5 = new ArrayList<>();
        internalPoints5.add(endTranslation5.plus(new Translation2d(Units.inchesToMeters(-24), Units.inchesToMeters(-5))));
        ArrayList<Translation2d> internalPoints6 = new ArrayList<>();
        internalPoints6.add(endTranslation6.plus(new Translation2d(Units.inchesToMeters(20),Units.inchesToMeters(5))));


        Command trajCommand = swerveDrive.generateTrajectory(startPose,endPose,internalPoints,0,0);
        Command trajCommand2 = swerveDrive.generateTrajectory(startPose2,endPose2,internalPoints2,0,0);
        Command trajCommand3 = swerveDrive.generateTrajectory(startPose3,endPose3,internalPoints3,0,0);
        Command trajCommand4 = swerveDrive.generateTrajectory(startPose4,endPose4,internalPoints4,0,0);
        Command trajCommand5 = swerveDrive.generateTrajectory(startPose5,endPose5,internalPoints5,0,0);
        Command trajCommand6 = swerveDrive.generateTrajectory(startPose6,endPose6,internalPoints6,0,0);

        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command shootLastNote = new shootSpeakerCommand(shooter, collector);
        Command shootAnotherLastNote = new shootSpeakerCommand(shooter, collector);

        Command collectNote = new Collect(collector,0.35,false);
        Command collectNote2 = new Collect(collector,0.35,false);
        Command collectNote3 = new Collect(collector,0.35,false);

        Command headingCorrect = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()-> AllianceFlip.apply(endRotation));
        Command headingCorrect2 = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()-> AllianceFlip.apply(endRotation3));
        Command headingCorrect4 = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()-> AllianceFlip.apply(endRotation4));
        Command headingCorrect6 = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()-> AllianceFlip.apply(endRotation4));

        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.runOnce(()->shooter.setShooterSpeeds(Constants.subShotSpeed,Constants.subShotSpeed)),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)).andThen(Commands.waitSeconds(.15)),
                Commands.race(Commands.waitSeconds(.2).andThen(shootNote),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(
                        Commands.parallel(
                                trajCommand.andThen(()->swerveDrive.stopModules()),
                                new Intake(collector, 0.35)
                        ),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))
                ).withTimeout(2),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.deadline(
                        trajCommand2.andThen(()->swerveDrive.stopModules()),
                        Commands.sequence(
                                new Intake(collector, 0.35),

                        ),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand3.andThen(()->swerveDrive.stopModules()), collectNote2),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(2),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(Commands.parallel(trajCommand4.andThen(()->swerveDrive.stopModules())),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(shootLastNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand5.andThen(()->swerveDrive.stopModules()), collectNote3),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(2),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(trajCommand6.andThen(()->swerveDrive.stopModules()),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(shootAnotherLastNote.andThen(()->shooter.stopShooter()), Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
        );
    }

    public Command DC3C2(){//TODO: Fix coordinates, create actual shoot and collect commands
        //go to C3 collect; go to ~W3; shoot; C2 collect; go to ~W3; shoot
        //traj 1
        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
        Rotation2d endRotation = new Rotation2d(0);
        Translation2d endTranslation = new Translation2d(UsefulPoints.Points.CenterNote3.getX()-Units.inchesToMeters(6),
                UsefulPoints.Points.CenterNote3.getY());
        Pose2d endPose = new Pose2d(endTranslation, endRotation); //goes from start D to C3


        Translation2d endTranslation2 = UsefulPoints.Points.StageEnterBottom;
//        Rotation2d startRotation2 = new Rotation2d(swerveDrive.getYaw());
        Pose2d startPose2 = new Pose2d(endTranslation, endRotation);
        Rotation2d endRotation2 = swerveDrive.getAngleBetweenSpeaker(endTranslation2);
        Pose2d endPose2 = new Pose2d(endTranslation2, endRotation2); //goes from point C3 to Shoot point

        Translation2d endTranslation3 = new Translation2d(UsefulPoints.Points.CenterNote2.getX(),
                UsefulPoints.Points.CenterNote2.getY() + Units.inchesToMeters(0));
        Rotation2d startRotation3 = endRotation2;
        Pose2d startPose3 = new Pose2d(endTranslation2, startRotation3);
        Rotation2d endRotation3 = new Rotation2d(0);
        Pose2d endPose3 = new Pose2d(endTranslation3,endRotation3); //shoot point to C2

        Translation2d endTranslation4 = endTranslation2;
        Rotation2d startRotation4 = endRotation3;
        Pose2d startPose4 = new Pose2d(endPose3.getTranslation(), startRotation4);
        Rotation2d endRotation4= (swerveDrive.getAngleBetweenSpeaker(endTranslation3).plus(Rotation2d.fromDegrees(-5)));
        Pose2d endPose4 = new Pose2d(endTranslation4, endRotation4); //C2 to shoot point


        Translation2d stageEntrancePoint = new Translation2d(UsefulPoints.Points.WingedNote3.getX(),UsefulPoints.Points.WingedNote3.getY()-Units.inchesToMeters(51));
        Translation2d stageEntranceBottom = UsefulPoints.Points.StageEnterBottom.plus(new Translation2d(Units.inchesToMeters(-6), 0));
        Translation2d stageExit = UsefulPoints.Points.OutOfStage;
        Translation2d stageMiddle = UsefulPoints.Points.CenterStage;

        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();
        ArrayList<Translation2d> internalPoints2 = new ArrayList<>();
        ArrayList<Translation2d> internalPoints3 = new ArrayList<>();
        ArrayList<Translation2d> internalPoints4 = new ArrayList<>();
        internalPoints.add(stageEntrancePoint);
        internalPoints.add(stageEntranceBottom);
        internalPoints.add(stageMiddle);
        internalPoints.add(stageExit);
        internalPoints2.add(stageExit);
        internalPoints2.add(stageMiddle);
        internalPoints3.add(stageMiddle);
        internalPoints3.add(stageExit);
        internalPoints4.add(stageExit);
        internalPoints4.add(stageMiddle);

        ArrayList <Pose2d> trajOnePoses = new ArrayList<>();
        trajOnePoses.add(startPose);
        trajOnePoses.add(new Pose2d(3.01, 2.15, Rotation2d.fromDegrees(15.48)));
        trajOnePoses.add(new Pose2d(4.77, 4.1, Rotation2d.fromDegrees(44.19)));
        trajOnePoses.add(new Pose2d(6.69, 4.22, Rotation2d.fromDegrees(-4.61)));
        trajOnePoses.add(endPose);

        Command trajCommand = swerveDrive.generateTrajectoryQuintic(trajOnePoses,0,0);
        Command trajCommand2 = swerveDrive.generateTrajectory(startPose2,endPose2,internalPoints2,0,0);
        Command trajCommand3 = swerveDrive.generateTrajectory(startPose3,endPose3,internalPoints3,0,0);
        Command trajCommand4 = swerveDrive.generateTrajectory(startPose4,endPose4,internalPoints4,0,0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector, 3000);
        Command shootLastNote = new shootSpeakerCommand(shooter, collector);

        Command collectNote = new Collect(collector,0.4,false);
        Command collectNote2 = new Collect(collector,0.4,false);

        Command headingCorrect2 = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()-> AllianceFlip.apply(endRotation2));
        Command headingCorrect4 = new setHeading(swerveDrive, ()-> 0.0, ()-> 0.0, ()-> AllianceFlip.apply(endRotation4));
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.runOnce(()->shooter.setShooterSpeeds(Constants.subShotSpeed,Constants.subShotSpeed)),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)).andThen(Commands.waitSeconds(.15)),
                Commands.race(Commands.waitSeconds(.2).andThen(shootNote),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(()->swerveDrive.stopModules()), collectNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(6),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(trajCommand2.andThen(()->swerveDrive.stopModules()),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.defer(()->armSubsystem.goToPoint(
                        Rotation2d.fromDegrees(armSubsystem.autoAim(swerveDrive::getEstimatedPose).getX()),
                        Rotation2d.fromDegrees(armSubsystem.autoAim(swerveDrive::getEstimatedPose).getY()-2)), Set.of(armSubsystem))
                        .andThen(Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())).withTimeout(1)),
                Commands.race(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(2),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)).andThen(Commands.waitSeconds(1)),
                Commands.race(Commands.parallel(trajCommand3.andThen(()->swerveDrive.stopModules()), collectNote2),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(5),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(Commands.parallel(trajCommand4.andThen(headingCorrect4.withTimeout(.25)).andThen(()->swerveDrive.stopModules())),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.defer(()->armSubsystem.goToPoint(
                        Rotation2d.fromDegrees(armSubsystem.autoAim(swerveDrive::getEstimatedPose).getX()),
                        Rotation2d.fromDegrees(armSubsystem.autoAim(swerveDrive::getEstimatedPose).getY()-2)), Set.of(armSubsystem)).andThen
                        (Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())).withTimeout(1)),
                Commands.parallel(shootLastNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
        );
    }

    public Command DC3ObjDetect(){//TODO: Fix coordinates, create actual shoot and collect commands
        Pose2d startPose = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
        Rotation2d endRotation = new Rotation2d(0);
        Translation2d finTranslation = UsefulPoints.Points.CenterNote3;
        Translation2d endTranslation = new Translation2d(UsefulPoints.Points.CenterNote3.getX()-Units.inchesToMeters(36),
                UsefulPoints.Points.CenterNote3.getY());
        Pose2d endPose = new Pose2d(endTranslation, endRotation); //goes from start D to C3


        Translation2d endTranslation2 = UsefulPoints.Points.StageEnterBottom;
        Pose2d startPose2 = new Pose2d(endTranslation, endRotation);
        Rotation2d endRotation2 = swerveDrive.getAngleBetweenSpeaker(endTranslation2);
        Pose2d endPose2 = new Pose2d(endTranslation2, endRotation2); //goes from point C3 to Shoot point

        ArrayList<Translation2d> internalPoints2 = new ArrayList<>();

        ArrayList <Pose2d> trajOnePoses = new ArrayList<>();
        trajOnePoses.add(startPose);
        trajOnePoses.add(new Pose2d(3.01, 2.15, Rotation2d.fromDegrees(15.48)));
        trajOnePoses.add(new Pose2d(4.77, 4.1, Rotation2d.fromDegrees(44.19)));
        //trajOnePoses.add(new Pose2d(6.69, 4.22, Rotation2d.fromDegrees(-4.61)));
        trajOnePoses.add(endPose);

        Command trajCommand = swerveDrive.generateTrajectoryQuintic(trajOnePoses,0,0);
        Command trajCommand2 = swerveDrive.generateTrajectory(startPose2,endPose2,internalPoints2,0,0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector, 3000);

        Command collectNote = new Collect(collector,0.4,false);

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

        return Commands.sequence(
                swerveDrive.setInitPosition(startPose),
                Commands.runOnce(()->shooter.setShooterSpeeds(Constants.subShotSpeed,Constants.subShotSpeed)),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)).andThen(Commands.waitSeconds(.15)),
                Commands.race(Commands.waitSeconds(.2).andThen(shootNote),Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(trajCommand.andThen(driveToNote.withTimeout(2))
                        .andThen(()->swerveDrive.stopModules()), collectNote),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(6),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.race(trajCommand2.andThen(()->swerveDrive.stopModules()),
                        Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.runOnce(()->swerveDrive.stopModules()),
                Commands.defer(()->armSubsystem.goToPoint(
                                Rotation2d.fromDegrees(armSubsystem.autoAim(swerveDrive::getEstimatedPose).getX()),
                                Rotation2d.fromDegrees(armSubsystem.autoAim(swerveDrive::getEstimatedPose).getY()-2)), Set.of(armSubsystem))
                        .andThen(Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())).withTimeout(1)),
                Commands.race(shootAnotherNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))).withTimeout(2)

        );
    }

    public Command EW3W2(){
        Rotation2d startRotation1 = Rotation2d.fromDegrees(0);
        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointE, startRotation1);
        Rotation2d endRotation1 = (swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote3));
        Pose2d endPose1 = new Pose2d(UsefulPoints.Points.WingedNote3, endRotation1);

//        Rotation2d startRotation2 = new Rotation2d(swerveDrive.getYaw());
        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.WingedNote3, endRotation1);

        Rotation2d endRotation2 = (swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote2));
        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.WingedNote2, endRotation2);
        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();

        Command traj1 = swerveDrive.generateTrajectory(startPose1,endPose1,internalPoints,0,0);
        Command traj2 = swerveDrive.generateTrajectory(startPose2,endPose2,internalPoints,0,0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command shootLastNote = new shootSpeakerCommand(shooter, collector);
        Command collectNote = new Collect(collector,1,false);
        Command collectNoteAgain = new Collect(collector,1,false);
        return Commands.sequence(
		        swerveDrive.setInitPosition(startPose1),
                shootNote,
				Commands.parallel(traj1, collectNote),
                shootAnotherNote,
		        Commands.parallel(traj2, collectNoteAgain),
                shootLastNote
        );
    }

    public Command BW1W2(){ //0 red collector; shoot blue
        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointB, UsefulPoints.Points.RotationOfStartingPointB);
        Rotation2d endRotation1 = (swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote1));
        Pose2d endPose1 = new Pose2d(UsefulPoints.Points.WingedNote1, endRotation1);

        Rotation2d startRotation2 = endRotation1;
        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.WingedNote1, startRotation2);
        Rotation2d endRotation2 = (swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote2));

        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.WingedNote2, endRotation2);
        Translation2d mid = new Translation2d(UsefulPoints.Points.WingedNote1.getX()-Units.inchesToMeters(50),
                UsefulPoints.Points.WingedNote1.getY()-Units.inchesToMeters(20));

        Translation2d mid2 = new Translation2d(UsefulPoints.Points.WingedNote2.getX()-Units.inchesToMeters(70),
                UsefulPoints.Points.WingedNote2.getY()+Units.inchesToMeters(10));

        //Translation2d mid = new Translation2d(60, 282.6);
        ArrayList<Translation2d> internalPoints1 = new ArrayList<Translation2d>();
        internalPoints1.add(mid);
        ArrayList<Translation2d> internalPoints2 = new ArrayList<Translation2d>();
        internalPoints2.add(mid2);


        Command traj1 = swerveDrive.generateTrajectory(startPose1, endPose1, internalPoints1,0,0);
        Command traj2 = swerveDrive.generateTrajectory(startPose2, endPose2, internalPoints2,0,0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootNote2 = new shootSpeakerCommand(shooter,collector);
        Command shootNote3 = new shootSpeakerCommand(shooter,collector);

        Command shootLastNote = new shootSpeakerCommand(shooter, collector);
        Command collectNote = new Collect(collector,.6,false);
        Command collectNote2 = new Collect(collector,.6,false);

        return Commands.sequence( //TODO: change values for arm and wrist
                swerveDrive.setInitPosition(startPose1),
                Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem)).andThen(Commands.waitSeconds(.15)),
                Commands.race(shootNote, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(traj1.andThen(()->swerveDrive.stopModules()), collectNote), Commands.run(()->armSubsystem.holdPos(85, -41))),
                Commands.runOnce(() -> swerveDrive.stopModules()),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(armSubsystem.autoAim(()->swerveDrive.getEstimatedPose()).getX()),
                        Rotation2d.fromDegrees(armSubsystem.autoAim(()->swerveDrive.getEstimatedPose()).getY())), Set.of(armSubsystem)),

                Commands.race(shootNote2.withTimeout(4), Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState()))),
                Commands.race(Commands.parallel(traj2.andThen(()->swerveDrive.stopModules()), collectNote2), Commands.run(()->armSubsystem.holdPos(85, -41))),
                Commands.runOnce(() -> swerveDrive.stopModules()),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(armSubsystem.autoAim(()->swerveDrive.getEstimatedPose()).getX()),
                        Rotation2d.fromDegrees(armSubsystem.autoAim(()->swerveDrive.getEstimatedPose()).getY())), Set.of(armSubsystem)),

                Commands.race(shootNote3, Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())))
        );
    }

    public Command DW3W2(){
        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
        Rotation2d endRotation1 = (swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote3));
        Pose2d endPose1 = new Pose2d(UsefulPoints.Points.WingedNote3, endRotation1);

        Rotation2d startRotation2 = new Rotation2d(swerveDrive.getYaw());
        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.WingedNote3, startRotation2);
        Rotation2d endRotation2 = (swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote2));

        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.WingedNote2, endRotation2);


        ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();

        Command traj1 = swerveDrive.generateTrajectory(startPose1, endPose1, internalPoints,0,0);
        Command traj2 = swerveDrive.generateTrajectory(startPose2, endPose2, internalPoints,0,0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootNote2 = new shootSpeakerCommand(shooter,collector);
        Command shootNote3 = new shootSpeakerCommand(shooter,collector);

        Command shootLastNote = new shootSpeakerCommand(shooter, collector);
        Command collectNote = new Collect(collector,1,false);
        Command collectNote2 = new Collect(collector,1,false);

        return Commands.sequence( //TODO: change values for arm and wrist
                swerveDrive.setInitPosition(startPose1),
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(85), Rotation2d.fromDegrees(-41)), Set.of(armSubsystem)),
                Commands.race(shootNote, Commands.run(()->armSubsystem.holdPos(85, -41))),
                Commands.race(Commands.parallel(traj1, collectNote), Commands.run(()->armSubsystem.holdPos(85, -41))),

                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(85), Rotation2d.fromDegrees(-41)), Set.of(armSubsystem)),
                Commands.race(shootNote2, Commands.run(()->armSubsystem.holdPos(85, -41))),
                Commands.race(Commands.parallel(traj2, collectNote2), Commands.run(()->armSubsystem.holdPos(85, -41))),

                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(85), Rotation2d.fromDegrees(-41)), Set.of(armSubsystem)),
                Commands.race(shootNote3, Commands.run(()->armSubsystem.holdPos(85, -41)))
        );
    }

    public Command DDetourBottomC5C4() {
        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointD, UsefulPoints.Points.RotationOfStartingPointD);
        Rotation2d endRotation1 = new Rotation2d(0);
        Pose2d endPose1 = new Pose2d(UsefulPoints.Points.CenterNote5, endRotation1);

        Rotation2d endRotation2 = (swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.DetourPointBottom));
        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.CenterNote5, new Rotation2d(0));
        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.DetourPointBottom, endRotation2);

        Pose2d startPose3 = new Pose2d(UsefulPoints.Points.DetourPointBottom, new Rotation2d(0));
        Pose2d endPose3 = new Pose2d(UsefulPoints.Points.CenterNote4, endRotation2);

        Pose2d startPose4 = new Pose2d(UsefulPoints.Points.CenterNote4, endRotation2);
        Pose2d endPose4 = new Pose2d(UsefulPoints.Points.DetourPointBottom, endRotation2);

        ArrayList<Translation2d> internalPoints1 = new ArrayList<Translation2d>();
        ArrayList<Translation2d> internalPoints2 = new ArrayList<Translation2d>();
        ArrayList<Translation2d> internalPoints3 = new ArrayList<Translation2d>();


        Command traj1 = swerveDrive.generateTrajectory(startPose1,endPose1, internalPoints1, 0,0);
        Command traj2 = swerveDrive.generateTrajectory(startPose2,endPose2, internalPoints2, 0,0);
        Command traj3 = swerveDrive.generateTrajectory(startPose3,endPose3, internalPoints3, 0,0);
        Command traj4 = swerveDrive.generateTrajectory(startPose4,endPose4, internalPoints3, 0,0);
        Command shootNote = new shootSpeakerCommand(shooter,collector);
        Command shootAnotherNote = new shootSpeakerCommand(shooter,collector);
        Command shootLastNote = new shootSpeakerCommand(shooter, collector);
        Command collectNote = new Collect(collector,1,false);
        Command collectNoteAgain = new Collect(collector,1,false);
        return Commands.sequence(
                swerveDrive.setInitPosition(startPose1),
                shootNote,
                traj1,
                Commands.parallel(traj2, collectNote),
                shootAnotherNote,
                Commands.parallel(traj3, collectNoteAgain),
                traj4,
                shootLastNote
        );
    }

    public Command BDetourTopC1C2(){
        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointB, UsefulPoints.Points.RotationOfStartingPointB);
        Rotation2d endRotation1 = new Rotation2d(0);
        Pose2d endPose1 = new Pose2d(UsefulPoints.Points.CenterNote1, endRotation1);

        Rotation2d endRotation2 = (swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.DetourPointTop));
        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.CenterNote1, new Rotation2d(0));
        Pose2d endPose2 = new Pose2d(UsefulPoints.Points.DetourPointTop, endRotation2);

        Pose2d startPose3 = new Pose2d(UsefulPoints.Points.DetourPointTop, new Rotation2d(0));
        Pose2d endPose3 = new Pose2d(UsefulPoints.Points.CenterNote2, endRotation2);

        Pose2d startPose4 = new Pose2d(UsefulPoints.Points.CenterNote2, endRotation2);
        Pose2d endPose4 = new Pose2d(UsefulPoints.Points.DetourPointTop, endRotation2);

        ArrayList<Translation2d> internalPoints1 = new ArrayList<Translation2d>();
        ArrayList<Translation2d> internalPoints2 = new ArrayList<Translation2d>();
        ArrayList<Translation2d> internalPoints3 = new ArrayList<Translation2d>();
	    ArrayList<Translation2d> internalPoints4 = new ArrayList<Translation2d>();


        Command traj1 = swerveDrive.generateTrajectory(startPose1,endPose1, internalPoints1, 0,0);
        Command traj2 = swerveDrive.generateTrajectory(startPose2,endPose2, internalPoints2, 0,0);
        Command traj3 = swerveDrive.generateTrajectory(startPose3,endPose3, internalPoints3, 0,0);
        Command traj4 = swerveDrive.generateTrajectory(startPose4,endPose4, internalPoints4, 0,0);

        return Commands.sequence(
                swerveDrive.setInitPosition(startPose1)
//                traj1
//                traj2,
//                traj3,
//                traj4
        );
    }



}
