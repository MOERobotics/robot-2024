// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.armSubsystem;
import frc.robot.subsystems.headSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PickupCommand extends Command {
    //as soon as sensor trips, stop collector.
    //collectingFromSourceState; neutralState (stowing away the note);
    // shootingPositionComannd, collcetingOffGround, depositAmp, depositTrap
    private final headSubsystem headSubsystem;
    private boolean hasNote;

    private ArmPosState armPos;




    public PickupCommand(headSubsystem headSubsystem, ArmPosState armPos) {
        this.headSubsystem = headSubsystem;
        this.armPos = armPos;
        addRequirements(headSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if(armPos //equal to neutral to collect from floor)
            armSubsystem.setDesiredArmState(); //to neutral
            headSubsystem.setCollectorSpeed(3000,3000);
        if(armPos //equal to collect from source)
            armSubsystem.setDesiredArmState(); //or source
            headSubsystem.setCollectorSpeed(3000,3000);

    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}