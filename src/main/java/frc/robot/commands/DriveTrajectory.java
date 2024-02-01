// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;

/** An example command that uses an example subsystem. */
public class DriveTrajectory extends Command {
    private Pose2d startPose = new Pose2d();
    private Pose2d endPose = new Pose2d();
    private ArrayList<Translation2d> internalPoints = new ArrayList<Translation2d>();

    private final double startVelocity;
    private final double endVelocity;
    Command trajectoryCommand;

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private final SwerveDrive m_subsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveTrajectory(SwerveDrive subsystem, Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints, double startVelocityMetersPerSecond, double endVelocityMetersPerSecond) {
        m_subsystem = subsystem;
        this.startPose = start;
        this.endPose = end;
        this.internalPoints = internalPoints;
        this.startVelocity = startVelocityMetersPerSecond;
        this.endVelocity = endVelocityMetersPerSecond;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putString("Initialized","initialized");
        trajectoryCommand = m_subsystem.generateTrajectory(startPose, endPose, internalPoints, startVelocity, endVelocity);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putString("Executing", Boolean.toString(trajectoryCommand.isFinished()));
        trajectoryCommand.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(trajectoryCommand == null){
            return true;
        } else {
            return trajectoryCommand.isFinished();
        }
    }
}
