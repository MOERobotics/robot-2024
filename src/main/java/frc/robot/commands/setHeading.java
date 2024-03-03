package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.Supplier;

public class setHeading extends Command {

    private Supplier<Rotation2d> desiredYaw;

    private SwerveDrive swerveDrive;
    private final Supplier<Double> xspdFunction, yspdFunction;

    private PIDController PID;

    public setHeading(SwerveDrive swerveDrive, Supplier<Double> xspeed, Supplier<Double> yspeed, Supplier<Rotation2d> desiredYaw){

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
        swerveDrive.setDesiredYaw(desiredYaw.get().getDegrees());
        double turnSpd = swerveDrive.getYawCorrection();
        swerveDrive.driveAtSpeed(xspdFunction.get(), yspdFunction.get(), turnSpd,true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return  Math.abs(MathUtil.inputModulus(swerveDrive.getYaw()-desiredYaw.get().getDegrees(),-180,180))<=0.5;//2 Degree Tolerance
    }




}
