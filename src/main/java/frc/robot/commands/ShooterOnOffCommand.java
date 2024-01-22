// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.headSubsystem;

/** An example command that uses an example subsystem. */
public class ShooterOnOffCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final headSubsystem m_subsystem;
    boolean onoff;
    boolean finished = false;
    double shooterSpeedTop;
    double shooterSpeedBottom;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ShooterOnOffCommand(headSubsystem subsystem, double shooterSpeedTop, double shooterSpeedBottom, boolean on) {
        m_subsystem = subsystem;
        this.shooterSpeedTop = shooterSpeedTop;
        this.shooterSpeedBottom = shooterSpeedBottom;
        onoff = on;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(onoff){
            m_subsystem.setShooterSpeed(shooterSpeedTop,shooterSpeedBottom);
        } else{
            m_subsystem.stopShooter();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(m_subsystem.getShooterSpeedBottom() >= shooterSpeedBottom && m_subsystem.getShooterSpeedTop() >= shooterSpeedTop){
            finished = true;
        } else if(!onoff){
            finished = true;
        }else{
            finished = false;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(finished){
            return true;
        }
        return false;
    }
}
