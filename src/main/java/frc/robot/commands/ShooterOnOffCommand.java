// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HeadSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterOnOffCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final HeadSubsystem headSubsystem;
    boolean onoff;
    boolean finished;
    double shooterSpeedTop;
    double shooterSpeedBottom;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ShooterOnOffCommand(HeadSubsystem subsystem, double shooterSpeedTop, double shooterSpeedBottom, boolean on) {
        headSubsystem = subsystem;
        this.shooterSpeedTop = shooterSpeedTop;
        this.shooterSpeedBottom = shooterSpeedBottom;
        onoff = on;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
		finished=false;
	    if(onoff){
//		    headSubsystem.setShooterTopSpeed(shooterSpeedTop);
//		    headSubsystem.setShooterBottomSpeed(shooterSpeedBottom);
			headSubsystem.setShooterPower(0.2);
	    } else{
//		    headSubsystem.stopShooter();
			headSubsystem.setShooterPower(0.0);
	    }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
	    if(onoff){
//		    headSubsystem.setShooterTopSpeed(shooterSpeedTop);
//		    headSubsystem.setShooterBottomSpeed(shooterSpeedBottom);
		    headSubsystem.setShooterPower(0.2);
	    } else{
//		    headSubsystem.stopShooter();
		    headSubsystem.setShooterPower(0.0);
	    }
//        if(headSubsystem.shooterAtSpeed()){
//            finished = true;
//        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
//        return finished;
	    return false;
    }
}
