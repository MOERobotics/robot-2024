package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberArm;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.Supplier;


public class testCommand extends Command{

    private final ClimberArm climberArm;

    private final Supplier<Boolean> rightBtn;
    private final Supplier<Boolean> leftBtn;


    public testCommand(ClimberArm climberArm, Supplier<Boolean> rightBtn, Supplier<Boolean> leftBtn ){


      this.climberArm = climberArm;

        this.rightBtn= rightBtn;
        this.leftBtn= leftBtn;


        addRequirements(climberArm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
       if(rightBtn.get()){
            climberArm.drive(0.3);
        } else if(leftBtn.get()){
           climberArm.drive(-0.3);
       } else {
           climberArm.drive(0);
       }

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
























