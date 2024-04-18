package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorSubsystem;

public class PopNote extends Command {
    private final double speed;
    private Timer timer;

    private CollectorSubsystem collector;
    public PopNote(CollectorSubsystem collector, double speed){
        this.collector=collector;
        this.speed=speed;
        timer = new Timer();
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double finalSpeed = -speed;
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
        if(timer.get() >= .1 || !collector.isCollected()){
            return true;
        } else {
            return false;
        }
    }
}