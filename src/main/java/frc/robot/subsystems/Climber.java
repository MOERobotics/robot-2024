package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {


    private Pigeon2 pigeon2;
    private final ClimberArm climberArmRight;
    private final ClimberArm climberArmLeft;


    private AHRS navx;

    public Climber(int climberIDRight, int climberIDLeft, int topLimitSwitchIDRight,  int topLimitSwitchLeft, int bottomLimitSwitchIDRight,  int bottomLimitSwitchLeft, int stringPotIDRight, int stringPotIDLeft) {


        climberArmRight = new ClimberArm(climberIDRight,/*topLimitSwitchIDRight,bottomLimitSwitchIDRight,*/ stringPotIDRight);
        climberArmLeft = new ClimberArm(climberIDLeft,/*topLimitSwitchLeft,bottomLimitSwitchLeft,*/ stringPotIDLeft);

    }


    public void driveRight(double speed){
        climberArmRight.drive(speed);
    }

    public void driveLeft(double speed){
        climberArmLeft.drive(speed);
    }

    public void stop(){
        climberArmRight.stop();
        climberArmRight.stop();

    }
    public void stopRight(){
        climberArmRight.stop();
    }

    public void stopLeft(){
        climberArmLeft.stop();
    }

    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
                () -> {
                    /* one-time action goes here */
                });
    }

    /**
     *
     * @return distance in meters,feet, ticks
     */
    public double getPositionDistanceRight() {
        return climberArmRight.getPositionDistance();
    }

    public double getPositionDistanceLeft() {
        return climberArmRight.getPositionDistance();
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
/*
    public boolean hasChainBottomRight(){
        return climberArmRight.hasChainBottom();
    }

    public boolean hasChainBottomLeft(){
        return climberArmLeft.hasChainBottom();
    }

    public boolean hasChainTopRight(){
        return climberArmRight.hasChainTop();
    }


    public boolean hasChainTopLeft(){
        return climberArmLeft.hasChainTop();
    }


 */
    public double getRoll(){

       return climberArmRight.getRoll();
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}