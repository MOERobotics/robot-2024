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
    private final HeadSubsystem m_subsystem;
    double[] shooterSpeed = new double[2];


    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public shootSpeakerCommand(HeadSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    //TODO: Calculate shooter speeds based on odometry.
    public double[] speedCalc(){
        return new double[]{0.0,0.0};
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //Set speed in RPM
        shooterSpeed = new double[]{3000,3000};//Placeholder
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.setShooterSpeed(shooterSpeed[0],shooterSpeed[1]);
        if(m_subsystem.readyShoot()){
            m_subsystem.setCollectorSpeed(200,200);
        }
        if(!m_subsystem.hasNote()){
            m_subsystem.stopCollector();
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopCollector();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //TODO: Need to fix the condition for when the command is finished.
        if(!m_subsystem.hasNote()){
            return true;
        }
        return false;
    }
}
