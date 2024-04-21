package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.subsystems.Climber;

/** An example command that uses an example subsystem. */
public class ClimbDown extends Command {

    private final Climber climber;

    private final double speed;




    public ClimbDown(Climber climber, double speed) {
        this.climber = climber;
        this.speed=speed;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.



    public void execute() {




        climber.driveLeft(speed);
        climber.driveRight(speed);

    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}