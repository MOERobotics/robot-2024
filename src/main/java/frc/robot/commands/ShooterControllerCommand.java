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
    Supplier<Boolean> onoff;
    Supplier<Double> desShoulder;
    boolean finished;
    double shooterSpeedTop;
    double shooterSpeedBottom;
    int onState = 0;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ShooterControllerCommand(ShooterSubsystem subsystem, Supplier <Double> desShoulder, Supplier<Boolean> on) {
        this.subsystem = subsystem;
        this.shooterSpeedTop = 5000;
        this.shooterSpeedBottom = 5000;
        this.desShoulder = desShoulder;
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
            shooterSpeedTop = 4000; shooterSpeedBottom = 4000;
            if (desShoulder.get() <= 85) {shooterSpeedTop = 3000; shooterSpeedBottom = 3000;}
            subsystem.setShooterSpeeds(shooterSpeedTop, shooterSpeedBottom);
        } else{
            shooterSpeedTop = 1000;
            shooterSpeedBottom = 1000;
            subsystem.setShooterSpeeds(shooterSpeedTop, shooterSpeedBottom);
            //subsystem.stopShooter();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}


}