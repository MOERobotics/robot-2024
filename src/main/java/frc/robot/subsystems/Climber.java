package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {



    private final ClimberArm climberArmRight;
    private final ClimberArm climberArmLeft;


    private AHRS navx;

    public Climber(int climberIDRight, int climberIDLeft, int stringPotIDRight, int stringPotIDLeft) {


        climberArmRight = new ClimberArm(climberIDRight, stringPotIDRight);
        climberArmLeft = new ClimberArm(climberIDLeft, stringPotIDLeft);

    }


    public void driveRight(double speed){
        climberArmRight.drive(speed);
    }

    public void driveLeft(double speed){
        climberArmLeft.drive(speed);
    }

    public void stop(){
        climberArmRight.stop();
        climberArmLeft.stop();

    }
    public void stopRight(){
        climberArmRight.stop();
    }

    public void stopLeft(){
        climberArmLeft.stop();
    }

    public double getPositionPercentRight() {
        return climberArmRight.getPositionPercent();
    }
    public double getPositionPercentLeft() {
        return climberArmLeft.getPositionPercent();
    }
    public double getPositionInchesRight() {
        return climberArmRight.getPositionInches();
    }
    public double getPositionInchesLeft() {
        return climberArmLeft.getPositionInches();
    }

    public boolean canGoUpRight(){
        return climberArmRight.canGoUp();
    }

    public boolean canGoUpLeft(){
        return climberArmRight.canGoUp();
    }



    public boolean canGoDownRight(){
        return  climberArmRight.canGoDown();
    }

    public boolean canGoDownLeft(){
        return  climberArmLeft.canGoDown();
    }

    public double getPositionVoltageRight(){
        return climberArmRight.getPositionVoltage();
    }
    public double getPositionVoltageLeft(){
        return climberArmLeft.getPositionVoltage();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Go up left?" , canGoUpLeft());
        SmartDashboard.putBoolean("Go down left?" , canGoDownRight());
        SmartDashboard.putBoolean("Go up right?" , canGoUpRight());
        SmartDashboard.putBoolean("Go down right?" , canGoDownRight());
        SmartDashboard.putNumber("String pot Voltage left:", getPositionVoltageLeft());
        SmartDashboard.putNumber("String pot Voltage right:", getPositionVoltageRight());
        SmartDashboard.putNumber("Speed left:", climberArmLeft.getSpeed());
        SmartDashboard.putNumber("Speed right:", climberArmRight.getSpeed());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}