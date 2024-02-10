// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HeadSubsystem;

import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class ShooterOnOffCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final HeadSubsystem headSubsystem;
    Supplier<Boolean> onoff;
    boolean finished;
    double shooterSpeedTop;
    double shooterSpeedBottom;
    int onState = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ShooterOnOffCommand(HeadSubsystem subsystem, double shooterSpeedTop, double shooterSpeedBottom, Supplier<Boolean> on) {
        headSubsystem = subsystem;
        this.shooterSpeedTop = shooterSpeedTop;
        this.shooterSpeedBottom = shooterSpeedBottom;
        onoff = on;
        onState = 0;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(onoff.get()){
            onState += 1;
            onState %= 2;
        }
        if (onState%2 == 1){
            headSubsystem.setShooterBottomSpeed(shooterSpeedBottom);
            headSubsystem.setShooterTopSpeed(shooterSpeedTop);
        } else{
            headSubsystem.setShooterBottomSpeed(0);
            headSubsystem.setShooterTopSpeed(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

}
