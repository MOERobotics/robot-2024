package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class shooterOn extends Command {

	private final ShooterSubsystem shooter;
	private final double speed;

	/**
	 * Creates a new shootOn.
	 *
	 * @param shooterSubsystem The shooter subsystem used by this command.
	 * @param speed  The speed in RPM to set the shooter motors to.
	 */
	public shooterOn(ShooterSubsystem shooterSubsystem, double speed) {
        this.shooter = shooterSubsystem;
		this.speed=speed;
    }
	@Override
	public void initialize(){
		shooter.setShooterSpeeds(speed,speed);
	}

	@Override
	public void execute() {
		shooter.setShooterSpeeds(speed,speed);
	}

	@Override
	public void end(boolean interrupted){
		if(interrupted){
			shooter.setShooterSpeeds(0,0);
		}
	}

	public boolean isFinished(){
		return true;
	}

}
