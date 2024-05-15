package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorSubsystem;

public class Intake extends Command {
    private final double speed;
    private CollectorSubsystem collector;
    public Intake(CollectorSubsystem collector, double speed){
        this.collector=collector;
        this.speed=speed;
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
        if (speed>0 && !collector.isCollected()) { //collecting in, we have no note
            finalSpeed = speed;
        }
        collector.updateCollectorSpeed(finalSpeed);
        SmartDashboard.putBoolean("started collector", collector.getCollectorState());
        SmartDashboard.putNumber("collector speed", finalSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        collector.stopCollector();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if(collector.isCollected()){
            return true;
        }
        return false;
    }
}