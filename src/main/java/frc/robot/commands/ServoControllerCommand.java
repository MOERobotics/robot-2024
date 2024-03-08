// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CollectorFinger;
import edu.wpi.first.wpilibj2.command.Command;


public class ServoControllerCommand extends Command {

	private final CollectorFinger subsystem;
	private double desiredPos;
	public ServoControllerCommand(double desiredPos, CollectorFinger subsystem) {
		this.subsystem = subsystem;
		this.desiredPos=desiredPos;
		addRequirements(subsystem);
	}


	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		subsystem.setServoPos(desiredPos);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}