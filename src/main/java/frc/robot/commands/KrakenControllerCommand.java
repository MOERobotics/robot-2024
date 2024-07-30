// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KrakenMotor;

import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class KrakenControllerCommand extends Command {
	private final KrakenMotor krakenMotor;
	private final Supplier<Double> speedSupply;
	public KrakenControllerCommand(KrakenMotor krakenMotor, Supplier <Double> speed) {
		this.krakenMotor = krakenMotor;
		speedSupply = speed;
		addRequirements(krakenMotor);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
//		System.out.printf("Speed Supply: %f\n", speedSupply.get());
//		System.out.flush();
		if(Math.abs(speedSupply.get())>0.1) {
			krakenMotor.set(speedSupply.get());
		}else{
			krakenMotor.set(0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		krakenMotor.stopMotor();
	}


}