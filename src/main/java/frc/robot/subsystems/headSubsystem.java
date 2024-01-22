// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class headSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private final CANSparkMax shooterTop;
    private final CANSparkMax shooterBottom;
    private final CANSparkMax collectorTop;
    private final CANSparkMax collectorBottom;
    private final RelativeEncoder shooterTopEncoder;
    private final RelativeEncoder shooterBottomEncoder;
    private final SparkPIDController shooterTopController;
    private final SparkPIDController shooterBottomController;
    private final DigitalInput collectorBeam;
    private double shooterSpeedTop=0;//Store desired speeds
    private double shooterSpeedBottom=0;
    public headSubsystem(int shooterTopID,int shooterBottomID,int collectorTopID,int collectorBottomID, double shooterP, double shooterI, double shooterD, int collectorBeamID) {
        //instantiate shooter motors, encoders, sensors, PID
        this.collectorBeam = new DigitalInput(collectorBeamID);
        this.shooterTop=new CANSparkMax(shooterTopID, CANSparkLowLevel.MotorType.kBrushless);
        this.shooterBottom=new CANSparkMax(shooterBottomID,CANSparkLowLevel.MotorType.kBrushless);
        this.collectorTop=new CANSparkMax(collectorTopID, CANSparkLowLevel.MotorType.kBrushless);
        this.collectorBottom=new CANSparkMax(collectorBottomID,CANSparkLowLevel.MotorType.kBrushless);
        shooterTop.setIdleMode(CANSparkBase.IdleMode.kCoast);
        shooterBottom.setIdleMode(CANSparkBase.IdleMode.kCoast);
        collectorTop.setIdleMode(CANSparkBase.IdleMode.kBrake);
        collectorBottom.setIdleMode(CANSparkBase.IdleMode.kBrake);

        this.shooterTopEncoder = shooterTop.getEncoder();
        this.shooterBottomEncoder = shooterBottom.getEncoder();

        this.shooterTopController = shooterTop.getPIDController();
        shooterTopController.setP(shooterP);
        shooterTopController.setI(shooterI);
        shooterTopController.setIZone(0);
        shooterTopController.setD(shooterD);
        shooterTopController.setOutputRange(-1, 1);
        this.shooterBottomController = shooterBottom.getPIDController();
        shooterBottomController.setP(shooterP);
        shooterBottomController.setI(shooterI);
        shooterBottomController.setIZone(0);
        shooterBottomController.setD(shooterD);
        shooterBottomController.setOutputRange(-1, 1);

    }



    //Has a note
    public boolean hasNote(){
        return collectorBeam.get();
    }

    public void setShooterTop(double speed){
        shooterSpeedTop=speed;
        shooterTopController.setReference(speed,CANSparkBase.ControlType.kVelocity);
    }
    public void setShooterBottom(double speed){
        shooterSpeedBottom=speed;
        shooterBottomController.setReference(speed,CANSparkBase.ControlType.kVelocity);
    }

    public void setCollectorSpeed(double topSpeed, double bottomSpeed){
        collectorTop.set(topSpeed);
        collectorBottom.set(bottomSpeed);
    }
    public double getShooterSpeedTop(){
        return shooterTopEncoder.getVelocity();
    }
    public double getShooterSpeedBottom(){
        return shooterBottomEncoder.getVelocity();
    }

    //Within reasonable range to shoot?
    public boolean inRange() {
        // Vision to April Tag/Odometry.
        return false;
    }

    //Shooter april tag seen
    public boolean seeSpeaker() {
        // Query Odometry or vision to April Tag.
        return false;
    }

    //Aimed at the speaker
    public boolean aimed() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    //Is in process of aiming?
    public boolean aiming() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }


    //Good to shoot
    public boolean readyShoot() {
        return (Math.abs(shooterTopEncoder.getVelocity() - shooterSpeedTop) <= 5 && Math.abs(shooterBottomEncoder.getVelocity() - shooterSpeedBottom) <= 5
                && aimed() && seeSpeaker());
        //if motors up to speed
        //if aimed
        //if see speaker
    }

    //Stop motors
    public void stopShooter(){
        setShooterTop(0);
        setShooterBottom(0);
    }


    public boolean readyDeposit(){
        return false;
    }

    public void stopMotors(){
        collectorBottom.set(0);
        collectorTop.set(0);
    }



    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterTopDesired:",shooterSpeedTop);
        SmartDashboard.putNumber("shooterBottomDesired:",shooterSpeedBottom);
        SmartDashboard.putNumber("shooterTopActualSpeed:",getShooterSpeedTop());
        SmartDashboard.putNumber("shooterBottomActualSpeed:",getShooterSpeedBottom());
        SmartDashboard.putBoolean("Ready to shoot:",readyShoot());
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}