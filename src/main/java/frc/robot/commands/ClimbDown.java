package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

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


        if(climber.canGoUpRight()){
            climber.driveRight(speed);
        } else {
            climber.driveRight(0);
        }

        if(climber.canGoUpLeft()){
            climber.driveLeft(speed);
        } else {
            climber.driveLeft(0);
        }


        /*
        double roll = rollSupplier.get() - initialRoll;
        double finalSpeed;
        if (climber.canGoDownLeft() && climber.canGoDownRight()) {
            if (roll < -tolerance) {
                finalSpeed = speed;
                climber.driveLeft(finalSpeed);
                climber.stopRight();
            } else if (roll > tolerance) {
                finalSpeed = speed;
                climber.stopLeft();
                climber.driveRight(finalSpeed);
            } else {
                finalSpeed = speed;
                climber.driveLeft(finalSpeed);
                climber.driveRight(finalSpeed);
            }
        } else if (climber.canGoDownLeft() || climber.canGoDownRight()) {
            if (roll < -tolerance) {
                climber.stopLeft();
                finalSpeed = -speed;
                climber.driveRight(finalSpeed);
            } else if (roll > tolerance) {
                finalSpeed = -speed;
                climber.driveLeft(finalSpeed);
                climber.stopRight();
            } else {
                climber.stopLeft();
                climber.stopRight();
            }
        } else {
            climber.stopLeft();
            climber.stopRight();
        }

     */


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