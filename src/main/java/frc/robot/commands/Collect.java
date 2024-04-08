// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class Collect extends Command {
    private final double speed;
    private boolean index;
    private boolean shouldStop = false;
    private CollectorSubsystem collector;
    private Timer timer;
    public Collect(CollectorSubsystem collector, double speed, boolean index){
        this.collector=collector;
        this.speed=speed;
        this.index=index;
        timer = new Timer();
        shouldStop = false;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double finalSpeed = 0;
        if(speed<0){
            finalSpeed=speed;
        }
        if (!shouldStop && speed>0 && ((index&&collector.isCollected())||!collector.isCollected())) { //collector in no note
            finalSpeed = speed;
            timer.restart();
        }
        if (collector.isCollected() && timer.get() <= .1 && speed > 0){
            shouldStop = true;
            finalSpeed = -speed;
        }
        collector.updateCollectorSpeed(finalSpeed);
        SmartDashboard.putBoolean("started collector", collector.getCollectorState());
        SmartDashboard.putNumber("collector speed", finalSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        collector.stopCollector();
        shouldStop = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(shouldStop && timer.get() >= .1){
            return true;
        }else if(index&&!collector.isCollected()){
            return true;
        }
        return false;
    }
}