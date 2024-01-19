// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.headSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CollectorOnOrOffCommand extends Command {
    //as soon as sensor trips, stop collector.
    //collectingFromSourceState; neutralState (stowing away the note);
    // shootingPositionComannd, collcetingOffGround, depositAmp, depositTrap
    private final headSubsystem headSubsystem;
    private final boolean onOff;



    public CollectorOnOrOffCommand(headSubsystem headSubsystem, boolean onOff) {
        this.headSubsystem = headSubsystem;
        this.onOff = onOff;
        addRequirements(headSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(!onOff){
            headSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}