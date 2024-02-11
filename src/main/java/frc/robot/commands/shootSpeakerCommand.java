// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class shootSpeakerCommand extends Command {
    //    private final HeadSubsystem headSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private  final CollectorSubsystem collectorSubsystem;
    /**
     * Creates a new ExampleCommand.
     *
     * @param shooterSubsystem The shooter subsystem used by this command.
     * @param collectorSubsystem The collector subsystem used by this command
     */
    public shootSpeakerCommand(ShooterSubsystem shooterSubsystem,CollectorSubsystem collectorSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.collectorSubsystem = collectorSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem,collectorSubsystem);
    }
    //TODO: Calculate shooter speeds based on odometry.
    public double speedCalc(){
        return 6000.0;
    }//placeholder value

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSubsystem.setShooterTopSpeed(speedCalc());
        shooterSubsystem.setShooterBottomSpeed(speedCalc());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        collectorSubsystem.stopCollector();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}