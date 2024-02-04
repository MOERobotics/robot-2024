// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HeadSubsystem;

/** An example command that uses an example subsystem. */
public class shootSpeakerCommand extends Command {
    private double shooterSpeedTop;
    private double shooterSpeedBottom;
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final HeadSubsystem headSubsystem;


    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public shootSpeakerCommand(HeadSubsystem subsystem) {
        headSubsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    //TODO: Calculate shooter speeds based on odometry.
    public double speedCalc(){
        return 0.0;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //Set speed in RPM
        shooterSpeedTop = speedCalc();//Placeholder
	    shooterSpeedBottom = speedCalc();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        headSubsystem.setShooterTopSpeed(shooterSpeedTop);
		headSubsystem.setShooterBottomSpeed(shooterSpeedBottom);
        if(headSubsystem.readyShoot()){
            headSubsystem.setCollectorSpeed(1);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        headSubsystem.stopCollector();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //TODO: Need to fix the condition for when the command is finished.
        if(!headSubsystem.isCollected()){
            return true;
        }
        return false;
    }
}
