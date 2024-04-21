// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helpers.LineHelpers;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmPathFollow extends Command {
    private final Arm armSubsystem;
    Translation2d desiredPoint;
    Translation2d startPoint, currPos;
    Timer timer;
    double targetDist, speed, accel, s, v_0;


    public ArmPathFollow(Arm subsystem, Rotation2d shoulderPos, Rotation2d wristPos, double speed, double accel) {
        armSubsystem = subsystem;
        addRequirements(subsystem);
        desiredPoint = new Translation2d(shoulderPos.getDegrees(), wristPos.getDegrees());
        currPos = new Translation2d(subsystem.shoulderState().getDegrees(), subsystem.wristState().getDegrees());
        this.speed = speed;
        this.accel = accel;
        timer = new Timer();
        s = 0;
    }

    @Override
    public void initialize() {
        System.out.println("initialized");
        SmartDashboard.putNumber("made it to init", timer.get());
        startPoint = new Translation2d(armSubsystem.shoulderState().getDegrees(), armSubsystem.wristState().getDegrees());
        targetDist = startPoint.getDistance(desiredPoint);
        v_0 = armSubsystem.getArmSpeed();
        timer.restart();
    }

    @Override
    public void execute() {

        s = LineHelpers.getS(desiredPoint.getDistance(startPoint), speed, accel, v_0, timer.get());
        double v = LineHelpers.getVel(desiredPoint.getDistance(startPoint), speed, accel, v_0, timer.get());
        var shoulderPos = (desiredPoint.getX()-startPoint.getX())/desiredPoint.getDistance(startPoint)*s+startPoint.getX();

        double wristPos = (desiredPoint.getY()-startPoint.getY())/(desiredPoint.getX()-startPoint.getX())*(armSubsystem.shoulderState().getDegrees()
                - startPoint.getX()) + startPoint.getY();

        double wristVel = (desiredPoint.getY()-startPoint.getY())/(desiredPoint.getDistance(startPoint))*v;
        double shoulderVel = (desiredPoint.getX()-startPoint.getX())/(desiredPoint.getDistance(startPoint))*v;

        if (Math.abs(desiredPoint.getX() - startPoint.getX()) <= 5){
            wristPos = (desiredPoint.getY()-startPoint.getY())/(desiredPoint.getDistance(startPoint))*s+startPoint.getY();
        }
        if (desiredPoint.getDistance(startPoint) <= 5){
            s = desiredPoint.getDistance(startPoint);
        }
        SmartDashboard.putNumber("ArmPathFollow writePos", wristPos);
        SmartDashboard.putNumber("ArmPathFollow shoulderPos", shoulderPos);
        SmartDashboard.putNumber("ArmPathFollow desiredWrist", desiredPoint.getY());
        SmartDashboard.putNumber("ArmPathFollow desiredShoulder", desiredPoint.getX());
        armSubsystem.pathFollow(Rotation2d.fromDegrees(shoulderPos), Rotation2d.fromDegrees(wristPos), wristVel, shoulderVel);

        armSubsystem.setWristDestState(wristPos);
        armSubsystem.setShoulderDesState(shoulderPos);
        currPos = new Translation2d(armSubsystem.shoulderState().getDegrees(), armSubsystem.wristState().getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("finished this command");
        System.out.println(s);
        armSubsystem.setWristDestState(desiredPoint.getY());
        armSubsystem.setShoulderDesState(desiredPoint.getX());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(s - startPoint.getDistance(desiredPoint)) <= .1;
        //return currPos.getDistance(desiredPoint) <= 5;
    }
}
