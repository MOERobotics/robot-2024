package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SmartDashboard.putNumber("ended", 0);

        swerveDrive.setDesiredYaw(desiredYaw.get().getDegrees());
        double turnSpd = swerveDrive.getYawCorrection();
        double xspd = xspdFunction.get();
        double yspd = yspdFunction.get();
        if (Math.abs(xspd) <= .3){
            xspd = 0;
        }
        if (Math.abs(yspd) <= .3){
            yspd = 0;
        }
        SmartDashboard.putNumber("turnspeedAuto", turnSpd);
        swerveDrive.driveAtSpeed(xspdFunction.get(), yspdFunction.get(), turnSpd,true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("ended", 1);
        swerveDrive.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return true;
        return  Math.abs(MathUtil.inputModulus(swerveDrive.getEstimatedPose().getRotation().getDegrees()-desiredYaw.get().getDegrees(),-180,180))<=2;//2 Degree Tolerance
    }




}
