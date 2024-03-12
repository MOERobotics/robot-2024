// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.UsefulPoints;
import frc.robot.commands.Collect;
import frc.robot.commands.shootSpeakerCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.Set;

public class tripleNoteAutos {

    private SwerveDrive swerveDrive;

    private final double bumperSize = 0;

    private final double startVelocity; //Velocities are in meters/second.
    private final double endVelocity;
    private ShooterSubsystem shooter;
    private CollectorSubsystem collector;
    private Arm armSubsystem;

    /** Example static factory for an autonomous command. */
    public tripleNoteAutos(SwerveDrive subsystem, Arm armSubsystem, ShooterSubsystem shooter, CollectorSubsystem collector, double startVelocity, double endVelocity) {
        swerveDrive=subsystem;
        this.armSubsystem = armSubsystem;
        this.startVelocity = startVelocity;
        this.endVelocity = endVelocity;
        this.shooter = shooter;
        this.collector = collector;
    }

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
        Rotation2d endRotation1 = (swerveDrive.getAngleBetweenSpeaker(wingedNote1));
        Pose2d endPose1 = new Pose2d(UsefulPoints.Points.WingedNote1, endRotation1);
        //shoot beginning note; collect winged 1 note
        //start traj 2
        Rotation2d startRotation2 = new Rotation2d(swerveDrive.getYaw());
        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.WingedNote1, startRotation2);

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

    public Command EW3W2(){
        Rotation2d startRotation1 = Rotation2d.fromDegrees(0);
        Pose2d startPose1 = new Pose2d(UsefulPoints.Points.StartingPointE, startRotation1);
        Rotation2d endRotation1 = (swerveDrive.getAngleBetweenSpeaker(UsefulPoints.Points.WingedNote3));
        Pose2d endPose1 = new Pose2d(UsefulPoints.Points.WingedNote3, endRotation1);

        Rotation2d startRotation2 = new Rotation2d(swerveDrive.getYaw());
        Pose2d startPose2 = new Pose2d(UsefulPoints.Points.WingedNote3, startRotation2);

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
                Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(85), Rotation2d.fromDegrees(-41)), Set.of(armSubsystem)),
                Commands.race(shootNote, Commands.run(()->armSubsystem.holdPos(85, -41))),
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
