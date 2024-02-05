package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.Supplier;

public class setHeading extends Command {

    private double desiredYaw;
    private double currentYaw;

    private SwerveDrive swerveDrive;
    private final double kP = 1e-1;
    private final double kD = 1e-4;
    private final double kI = 0;
    private final Supplier<Double> xspdFunction, yspdFunction;

    private PIDController PID;

    public setHeading(SwerveDrive swerveDrive, Supplier<Double> xspeed, Supplier<Double> yspeed, double desiredYaw){
        PID = new PIDController(kP,kI,kD);
        PID.enableContinuousInput(-180,180);
        xspdFunction = xspeed;
        yspdFunction = yspeed;
        this.swerveDrive = swerveDrive;
        this.desiredYaw = desiredYaw;

        addRequirements(swerveDrive);

    }


    @Override
    public void initialize() {
    }



    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double turnSpd = PID.calculate(swerveDrive.getYaw(),desiredYaw);
        swerveDrive.driveAtSpeed(xspdFunction.get(), yspdFunction.get(), turnSpd,true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  Math.abs(swerveDrive.getYaw()-desiredYaw)<=0.5;//2 Degree Tolerance
    }




}
