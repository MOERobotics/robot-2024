// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class ShooterControllerCommand extends Command {
    private final ShooterSubsystem subsystem;
    Supplier<Boolean> toggle;
    double shooterSpeedTop;
    double shooterSpeedBottom;
    int onState = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ShooterControllerCommand(ShooterSubsystem subsystem, double shooterSpeedTop, double shooterSpeedBottom, Supplier<Boolean> toggle) {
        this.subsystem = subsystem;
        this.shooterSpeedTop = shooterSpeedTop;
        this.shooterSpeedBottom = shooterSpeedBottom;
        this.toggle = toggle;
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
        if(toggle.get()){
            onState += 1;
            onState %= 2;
        }
        if (onState%2 == 1){
            subsystem.setShooterSpeeds(shooterSpeedTop, shooterSpeedBottom);
        } else{
            subsystem.stopShooter();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished(){
        return subsystem.shooterAtSpeed();
    }

}