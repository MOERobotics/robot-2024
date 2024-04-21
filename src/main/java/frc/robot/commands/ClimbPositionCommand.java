package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimbPositionCommand extends Command {
    private final Climber climber;
    private final double speed;

    private final double  tolerance = 0.5;

    private final double endPositionInches;

    public ClimbPositionCommand(Climber climber, double speed, double endPositionInches) {
        this.endPositionInches = endPositionInches;
        this.climber = climber;
        this.speed=speed;
        addRequirements(climber);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double positionInchesRight = climber.getPositionInchesRight();
        double positionInchesLeft = climber.getPositionInchesLeft();
        double finalspeed;
        if(positionInchesRight < endPositionInches + tolerance && climber.canGoUpRight()){
            climber.driveRight(speed);
        } else if (positionInchesRight > endPositionInches && climber.canGoDownRight()) {
           finalspeed= -speed;
            climber.driveRight(finalspeed);
        } else {
            climber.stopRight();
        }

        if(positionInchesLeft < endPositionInches + tolerance && climber.canGoUpLeft()){
            climber.driveLeft(speed);
        } else if (positionInchesLeft > endPositionInches - tolerance && climber.canGoDownLeft()) {
            finalspeed= -speed;
            climber.driveLeft(finalspeed);
        } else {
            climber.stopLeft();
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