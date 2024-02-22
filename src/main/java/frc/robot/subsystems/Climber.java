package frc.robot.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {


    private Pigeon2 pigeon2;
    private ClimberArm climberArmRight;
    private ClimberArm climberArmLeft;


    public Climber() {


        climberArmRight = new ClimberArm(90,99,98, pigeon2 );
        climberArmLeft = new ClimberArm(90,99,98, pigeon2 );



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



    public double getRoll(){

       return climberArmRight.getRoll();
    }
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
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