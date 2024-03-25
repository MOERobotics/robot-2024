// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class NoteFeed extends Command {
	//    private final HeadSubsystem headSubsystem;
	private final ShooterSubsystem shooter;
	private  final CollectorSubsystem collector;
	private final Timer timer;
	private boolean shot = false;
	private final Supplier<Integer>shooterRpm;
	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param shooterSubsystem The shooter subsystem used by this command.
	 * @param collectorSubsystem The collector subsystem used by this command
	 * @param shooterRPM The supplier for the RPM which shooters should be set to
	 */
	public NoteFeed(ShooterSubsystem shooterSubsystem, CollectorSubsystem collectorSubsystem, Supplier<Integer> shooterRPM) {
		this.shooter = shooterSubsystem;
		this.collector = collectorSubsystem;
		timer = new Timer();
		this.shooterRpm=shooterRPM;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(shooterSubsystem,collectorSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		shot=false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		shooter.setShooterTopSpeed(shooterRpm.get());
		shooter.setShooterBottomSpeed(shooterRpm.get());
		if(shooter.shooterAtSpeed() && !shot) {
			shot = true;
			collector.setCollectorSpeed(1);
			timer.restart();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		shooter.stopShooter();
		collector.stopCollector();
		shot=false;
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return (shot && timer.get() >= .75);
	}
}