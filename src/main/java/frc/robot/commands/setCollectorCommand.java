// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.headSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class setCollectorCommand extends Command {
   //as soon as sensor trips, stop collector.
    //collectingFromSourceState; neutralState (stowing away the note);
   // shootingPositionComannd, collcetingOffGround, depositAmp, depositTrap
    private final headSubsystem headSubsystem;
    private final double speed;



    public setCollectorCommand(headSubsystem headSubsystem, double speed) {
        this.headSubsystem = headSubsystem;
        this.speed = speed;
        addRequirements(headSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
     headSubsystem.setShooterTop(speed);
     headSubsystem.setShooterBottom(speed);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}