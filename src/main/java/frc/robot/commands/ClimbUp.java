package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class ClimbUp extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Climber climber;

    private int tolerance;

    private final double speed;

    private double initialRoll;


    private final Supplier<Double> rollSupplier;


    public ClimbUp(Climber climber, double speed, Supplier<Double> rollSupplier) {
        this.rollSupplier = rollSupplier;
        this.climber = climber;
        this.speed=speed;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    public void initialize() {
         initialRoll = rollSupplier.get();
    }

    // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        double roll = rollSupplier.get() - initialRoll;
        double finalSpeed;

        if (climber.canGoUpLeft() && climber.canGoUpRight()) {

            if (roll < -tolerance) {
                climber.stopLeft();
                finalSpeed =-speed;
                climber.driveRight(finalSpeed);
            } else if (roll > tolerance) {
                finalSpeed =-speed;
                climber.driveLeft(finalSpeed);
                climber.stopRight();
            } else {
                finalSpeed =-speed;
                climber.driveLeft(finalSpeed);
                climber.driveRight(finalSpeed);
            }
        } else if (climber.canGoUpLeft() || climber.canGoUpRight()) {
            if (roll < -tolerance) {
                climber.driveLeft(speed);
                climber.stopRight();
            } else if (roll > tolerance) {
                climber.stopLeft();
                climber.driveRight(speed);
            } else {
                climber.stopLeft();
                climber.stopRight();
            }
        } else {
            climber.stopLeft();
            climber.stopRight();
        }

    }

    // Called once the command ends or is interrupted.

    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    public boolean isFinished() {
        return false;
    }
}