// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.helpers.LineHelpers;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmPathFollow extends Command {
    private final Arm armSubsystem;
    Translation2d desiredPoint;
    Translation2d startPoint;
    Timer timer;
    double targetDist, speed, accel;

    public ArmPathFollow(Arm subsystem, Rotation2d shoulderPos, Rotation2d wristPos, double speed, double accel) {
        armSubsystem = subsystem;
        addRequirements(subsystem);
        desiredPoint = new Translation2d(shoulderPos.getDegrees(), wristPos.getDegrees());
        this.speed = speed;
        this.accel = accel;
    }

    @Override
    public void initialize() {
        startPoint = new Translation2d(armSubsystem.shoulderState().getDegrees(), armSubsystem.wristState().getDegrees());
        targetDist = startPoint.getDistance(desiredPoint);
        timer.restart();
    }

    @Override
    public void execute() {
        double s = LineHelpers.getS(targetDist, speed, accel, timer.get());
        double shoulderPos = LineHelpers.getPositionX(startPoint, desiredPoint, s);
        double wristPos = LineHelpers.getPositionY(startPoint, desiredPoint, s);
        armSubsystem.pathFollow(Rotation2d.fromDegrees(shoulderPos), Rotation2d.fromDegrees(wristPos));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return startPoint.getDistance(desiredPoint) <= .2;
    }
}
