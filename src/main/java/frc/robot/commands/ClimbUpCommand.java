package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimbUpCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber climber;

    private int tolerance;

    private final double speed;

    private double intialRoll;

    public ClimbUpCommand(Climber climber, double speed) {
        this.climber = climber;
        this.speed=speed;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
         intialRoll = climber.getRoll();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double roll = climber.getRoll();
        double finalSpeed;

        if (climber.canGoUpRight() && climber.canGoUpLeft()) {
            climber.driveLeft(speed);
            climber.driveRight(speed);
        } else if (roll > intialRoll) {
            climber.stopLeft();
            if (climber.canGoDownRight()) {
                finalSpeed = -speed;
               climber.driveRight(finalSpeed);
            } else {
                climber.stopRight();
            }
        } else if (roll < -intialRoll) {
            climber.stopRight();
            if (climber.canGoDownLeft()) {
                finalSpeed = -speed;
               climber.driveLeft(finalSpeed);
            } else {
                climber.stopLeft();
            }
        } else {
            climber.stopLeft();
            climber.stopRight();
        }

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