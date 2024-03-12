// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CollectorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class CollectorControllerCommand extends Command {


    private final double speed;
    private final Supplier<Boolean> buttonOut;
    private final Supplier<Boolean> buttonIn;
    private final CollectorSubsystem subsystem;
    private final Supplier<Boolean> index;
    private final Timer timer;






    public CollectorControllerCommand(double speed, Supplier<Boolean> buttonOut,
                                      Supplier<Boolean> buttonIn, Supplier<Boolean> index, CollectorSubsystem subsystem) {
        this.speed = speed;
        this.index = index;
        this.buttonOut = buttonOut;
        this.buttonIn = buttonIn;
        this.subsystem = subsystem;
        timer = new Timer();

        addRequirements(subsystem);

    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double finalSpeed = 0;

        if (buttonOut.get() && !buttonIn.get()) { //collector out
            finalSpeed = -speed;
        }
        if (!buttonOut.get() && buttonIn.get() && !subsystem.isCollected()) { //collector in no note
            finalSpeed = speed;
            timer.restart();
        }
        if (subsystem.isCollected() && timer.get() <= .05){
            finalSpeed = -speed;
        }
        if(index.get()){
            finalSpeed = 1;
        }

        subsystem.updateCollectorSpeed(finalSpeed);
        SmartDashboard.putBoolean("started collector", subsystem.getCollectorState());
        SmartDashboard.putNumber("collector speed", finalSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}