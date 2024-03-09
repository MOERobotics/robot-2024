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

import static java.lang.Math.abs;

/** An example command that uses an example subsystem. */
public class ArmPathFollow extends Command {
    private final Arm armSubsystem;
    Translation2d desiredPoint;
    Translation2d startPoint, currPos;
    Timer timer;
    double targetDist, speed, accel, s;


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
        SmartDashboard.putNumber("made it to init", timer.get());
        startPoint = new Translation2d(armSubsystem.shoulderState().getDegrees(), armSubsystem.wristState().getDegrees());
        targetDist = startPoint.getDistance(desiredPoint);
        timer.restart();
    }

    @Override
    public void execute() {
        //SmartDashboard.putNumber("made it", timer.get());
        //s = LineHelpers.getS(targetDist, speed, accel, timer.get());
        //SmartDashboard.putNumber("ArmPathFollow s", s);
        //s = Math.min(timer.get()*speed, startPoint.getDistance(desiredPoint));
        s = Math.min(timer.get()*speed, desiredPoint.getDistance(startPoint));
        //var interPos = startPoint.getDistance(desiredPoint)
        //var interPos = startPoint.interpolate(desiredPoint, s / startPoint.getDistance(desiredPoint));
//        double shoulderPos = LineHelpers.getPositionX(startPoint, desiredPoint, s);
//        double wristPos = LineHelpers.getPositionY(startPoint, desiredPoint, s);
        var shoulderPos = (desiredPoint.getX()-startPoint.getX())/desiredPoint.getDistance(startPoint)*s+startPoint.getX();

        double wristPos = (desiredPoint.getY()-startPoint.getY())/(desiredPoint.getX()-startPoint.getX())*(armSubsystem.shoulderState().getDegrees()
                - startPoint.getX()) + startPoint.getY();
        if (Math.abs(desiredPoint.getX() - startPoint.getX()) <= 5){
            wristPos = (desiredPoint.getY()-startPoint.getY())/(desiredPoint.getDistance(startPoint))*s+startPoint.getY();
        }
        if (desiredPoint.getDistance(startPoint) <= 5){
            s = desiredPoint.getDistance(startPoint)+1;
        }
        SmartDashboard.putNumber("ArmPathFollow writePos", wristPos);
        SmartDashboard.putNumber("ArmPathFollow desiredWrite", desiredPoint.getY());
        armSubsystem.pathFollow(Rotation2d.fromDegrees(shoulderPos), Rotation2d.fromDegrees(wristPos));
        armSubsystem.setWristDestState(wristPos);
        armSubsystem.setShoulderDesState(shoulderPos);
        currPos = new Translation2d(armSubsystem.shoulderState().getDegrees(), armSubsystem.wristState().getDegrees());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.setWristDestState(desiredPoint.getY());
        armSubsystem.setShoulderDesState(desiredPoint.getX());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return s >= startPoint.getDistance(desiredPoint);
        //return currPos.getDistance(desiredPoint) <= 5;
    }
}
