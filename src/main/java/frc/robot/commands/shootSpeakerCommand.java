// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class shootSpeakerCommand extends Command {
    private final ShooterSubsystem shooter;
    private  final CollectorSubsystem collector;
	private final double speed;
    private Timer timer;
    private boolean shot = false;
    /**
     * Creates a new shootSpeakerCommand.
     *
     * @param shooterSubsystem The shooter subsystem used by this command.
     * @param collectorSubsystem The collector subsystem used by this command.
     * @param speed  The speed in RPM to set the shooter motors to.
     */
    public shootSpeakerCommand(ShooterSubsystem shooterSubsystem,CollectorSubsystem collectorSubsystem, double speed) {
        this.shooter = shooterSubsystem;
        this.collector = collectorSubsystem;
		this.speed=speed;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem,collectorSubsystem);

        timer = new Timer();
        shot = false;
    }
	public shootSpeakerCommand(ShooterSubsystem shooterSubsystem,CollectorSubsystem collectorSubsystem) {
		this(shooterSubsystem,collectorSubsystem, Constants.subShotSpeed);
	}
    //TODO: Calculate shooter speeds based on odometry.

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shot = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setShooterTopSpeed(speed);
        shooter.setShooterBottomSpeed(speed);
        if(shooter.shooterAtSpeed() && !shot){
            System.out.println("at speed, let's shoot");
            shot = true;
            collector.setCollectorSpeed(1);
            timer.restart();
        }
        SmartDashboard.putNumber("auto Time", timer.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //shooter.stopShooter();
        collector.stopCollector();
        shot = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (shot && timer.get() >= .25);
    }
}