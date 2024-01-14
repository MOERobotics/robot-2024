// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerveDrive;

import java.util.function.Supplier;


public class swerveController extends Command{

    private final swerveDrive m_subsystem;
    private final Supplier<Double> xspdFunction, yspdFunction, turnspdFunction;
    private final Supplier<Boolean> half;
    private final Supplier<Boolean> relativeDrive;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
    private final double maxMPS, maxRPS;

    public swerveController(swerveDrive subsystem, Supplier<Double> xspeed, Supplier<Double> yspeed,
                             Supplier<Double> turnspeed, Supplier<Boolean> slow, Supplier<Boolean> relative,
                            double maxAccel, double maxAngAccel, double maxMPS, double maxRPS) {
        m_subsystem = subsystem;
        xspdFunction = xspeed;
        yspdFunction = yspeed;
        turnspdFunction = turnspeed;
        half = slow;

        relativeDrive = relative;

        xLimiter = new SlewRateLimiter(maxAccel);
        yLimiter = new SlewRateLimiter(maxAccel);
        turnLimiter = new SlewRateLimiter(maxAngAccel);
        this.maxMPS = maxMPS;
        this.maxRPS = maxRPS;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double div = 1.0;
        if (half.get()) div = 2.0;
        double xspd = xspdFunction.get()/div;
        double yspd = yspdFunction.get()/div;
        double turnspd = turnspdFunction.get()/div;

        if (Math.abs(xspd) <= .3){
            xspd = 0;
        }
        if (Math.abs(yspd) <= .3){
            yspd = 0;
        }
        if (Math.abs(turnspd) <= .3){
            turnspd = 0;
        }
        xspd = xLimiter.calculate(xspd)*maxMPS;
        yspd = yLimiter.calculate(yspd)*maxMPS;
        turnspd = turnLimiter.calculate(turnspd)*maxRPS * 2.0;

        SmartDashboard.putNumber("xspd", xspd);
        SmartDashboard.putNumber("yspd", yspd);
        SmartDashboard.putNumber("turnspd", turnspd);

        m_subsystem.driveAtSpeed(xspd, yspd, turnspd, !relativeDrive.get());

    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}























