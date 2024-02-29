package frc.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

import java.util.function.Supplier;


public class TestClimber extends Command{

    private final Climber climber;

    private final Supplier<Boolean> upRightBtn;
    private final Supplier<Boolean> downRightBtn;
    private final Supplier<Boolean> upLeftBtn;

    private final Supplier<Boolean> downLeftBtn;

    public TestClimber(Climber climber, Supplier<Boolean> upLeftBtn, Supplier<Boolean> downLeftBtn, Supplier<Boolean> upRightBtn, Supplier<Boolean> downRightBtn ){
        this.climber = climber;

        this.upLeftBtn= upLeftBtn;
        this.upRightBtn= upRightBtn;

        this.downLeftBtn = downLeftBtn;
        this.downRightBtn= downRightBtn;
        addRequirements(climber);
    }

    @Override
    public void execute() {
       if(upRightBtn.get()){
            climber.driveRight(0.3);
       } else if(downRightBtn.get()){
           climber.driveRight(-0.3);
       } else {
           climber.stopRight();
       }

       if(upLeftBtn.get()){
           climber.driveLeft(0.3);
       }else if(downLeftBtn.get()){
           climber.driveLeft(-0.3);
       }else {
           climber.stopLeft();
       }

    }

}
























