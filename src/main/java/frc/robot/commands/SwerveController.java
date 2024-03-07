// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.Supplier;


public class SwerveController extends Command{

    private final SwerveDrive m_subsystem;
    private final Supplier<Double> xspdFunction, yspdFunction, turnspdFunction;
    private final Supplier<Boolean> half;
    private final Supplier<Boolean> relativeDrive;
    private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
    private final double maxMPS, maxRPS;

    public SwerveController(SwerveDrive subsystem, Supplier<Double> xspeed, Supplier<Double> yspeed,
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
    public void initialize() {
    }

    @Override
    public void execute() {
        double div = 1.0;
        if (half.get()) div = 2.0;
        double xspd = xspdFunction.get()/div;
        double yspd = yspdFunction.get()/div;
        double turnspd = turnspdFunction.get()/div;

        double D = .1;
        double G = .5;

        if (Math.abs(xspd) <= D){
            xspd = 0;
        }
        else{
            xspd = (xspd-Math.signum(yspd)*D)/(1-D);
            xspd = (G*Math.pow(xspd, 3) + (1-G)*xspd)*maxMPS;
        }
        if (Math.abs(yspd) <= D){
            yspd = 0;
        }
        else{
            yspd = (yspd-Math.signum(yspd)*D)/(1-D);
            yspd = (G*Math.pow(yspd, 3) + (1-G)*yspd)*maxMPS;
        }
        if (Math.abs(turnspd) < D){
            turnspd = 0;
        }
        else{
            turnspd = (turnspd-Math.signum(yspd)*D)/(1-D);
            turnspd = (G*Math.pow(turnspd, 3) + (1-G)*turnspd)*maxMPS;
        }


        SmartDashboard.putNumber("xspd", xspd);
        SmartDashboard.putNumber("yspd", yspd);
        SmartDashboard.putNumber("turnspd", turnspd);
        if (xspd == 0 && yspd == 0 && turnspd == 0){
            m_subsystem.stopModules();
        }
        else {
            m_subsystem.driveAtSpeed(xspd, yspd, turnspd, !relativeDrive.get());
        }
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























