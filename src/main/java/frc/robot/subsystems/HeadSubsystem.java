// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HeadSubsystem extends SubsystemBase {
//<<<<<<<< HEAD:src/main/java/frc/robot/subsystems/HeadSubsystem.java
    /** Creates a new ExampleSubsystem. */
    private final CANSparkMax shooterTop;
    private final CANSparkMax shooterBottom;
    private final CANSparkMax collectorID;
    private final RelativeEncoder shooterTopEncoder;
    private final RelativeEncoder shooterBottomEncoder;
    private final SparkPIDController shooterTopController;
    private final SparkPIDController shooterBottomController;

    private final SparkPIDController collectorController;
    private final DigitalInput collectorBeam;
    private double shooterSpeedTop=0;//Store desired speeds
    private double shooterSpeedBottom=0;

    private boolean collectorState;
    double height = 20;

    private double horizontalDistance;
    private double verticalDistance;

    private int shooterRPMtolerance;
    // TODO rename tolerance to explain what it is (shooterRPMtolerance)(done)
    public HeadSubsystem(int shooterTopID, int shooterBottomID, int collectorID, double shooterP, double shooterI, double shooterD, double shooterFF,
                         double collectorP, double collectorI, double collectorD, double collectorFF,  int collectorBeamID) {
        // TODO create PID for bottom controller(done)
        // TODO make collector PID controlled(done)
        //instantiate shooter motors, encoders, sensors, PID
        this.collectorBeam = new DigitalInput(collectorBeamID);
        this.shooterTop=new CANSparkMax(shooterTopID, CANSparkLowLevel.MotorType.kBrushless);
        this.shooterBottom=new CANSparkMax(shooterBottomID,CANSparkLowLevel.MotorType.kBrushless);
        this.collectorID=new CANSparkMax(collectorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterTop.setIdleMode(CANSparkBase.IdleMode.kCoast);
        shooterBottom.setIdleMode(CANSparkBase.IdleMode.kCoast);
        collectorController.setIdleMode(CANSparkBase.IdleMode.kBrake);

        this.shooterTopEncoder = shooterTop.getEncoder();
        this.shooterBottomEncoder = shooterBottom.getEncoder();

        // configure collector motor top and bottom

        // set top shooter PID
        this.shooterTopController = shooterTop.getPIDController();
        shooterTopController.setP(shooterP);
        shooterTopController.setI(shooterI);
        shooterTopController.setIZone(0);
        shooterTopController.setD(shooterD);
        shooterTopController.setFF(shooterFF);
        shooterTopController.setOutputRange(-1, 1);

        // set bottom shooter PID
        this.shooterBottomController = shooterBottom.getPIDController();
        shooterBottomController.setP(shooterP);
        shooterBottomController.setI(shooterI);
        shooterBottomController.setIZone(0);
        shooterBottomController.setD(shooterD);
        shooterBottomController.setFF(shooterFF);
        shooterBottomController.setOutputRange(-1, 1);

        // set Collecter PID
        this.collectorController = collectorID.getPIDController();
        collectorController.setP(collectorP);
        collectorController.setI(collectorI);
        collectorController.setIZone(0);
        collectorController.setD(collectorD);
        collectorController.setFF(collectorFF);
        collectorController.setOutputRange(-1, 1);


        // sets up tolerance
        setShooterRPMtolerance(5);

    }

    public boolean hasNote(){
        return collectorBeam.get();
    }

    //Has a note

    // TODO rename methods(partially done?)
    public void setShooterTopSpeed(double speed){
        shooterSpeedTop=speed;
        shooterTopController.setReference(speed,CANSparkBase.ControlType.kVelocity);
    }
    public void setShooterBottomSpeed(double speed){
        shooterSpeedBottom=speed;
        shooterBottomController.setReference(speed,CANSparkBase.ControlType.kVelocity);
    }
    // TODO check collector speeds before turning on(check)
    public void setCollectorSpeed(double speed){
        collectorID.set(speed);
        if(speed == 0){
            collectorState = false;
        } else{
            collectorState = true;
        }
    }

    public Command runCollectorCommands (double speed){
        return  Commands.runOnce(() -> this.setCollectorSpeed(speed));
    }

    //TODO Make collector state of on or off(done)
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

        // TODO feed in variable for shooter tolerance and overload method(done)
        //if motors up to speed
        //if aimed
        //if see speaker
    }

    public double getAngleBetweenSpeaker(Pose2d pose){
        if(readyShoot() && seeSpeaker()){
            return Math.atan(pose.getY()/pose.getX());
        }
        return 0;
    }




    public void setShooterRPMtolerance(int shooterRPMtolerance){

        this.shooterRPMtolerance = shooterRPMtolerance;

    }

    public int getShooterRPMtolerance(){
        return shooterRPMtolerance;
    }


    //Stop motors
    public void stopShooter(){
        setShooterTopSpeed(0);
        setShooterBottomSpeed(0);
    }


    public boolean readyDeposit(){
        return false;
    }

    public void stopCollector(){
        collectorID.set(0);

    }


    public boolean getCollectorState (){
        return collectorState;
    }



    // TODO A state where we know its safe to move even if the note isn't completely in the head(not done)

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

  //Has a note

  public void setShooterSpeed(double speedTop,double speedBottom){
    shooterSpeedTop=speedTop;
    shooterSpeedBottom = speedBottom;
    shooterTopController.setReference(speedTop,CANSparkBase.ControlType.kVelocity);
    shooterBottomController.setReference(speedBottom,CANSparkBase.ControlType.kVelocity);
  }


  public double calculateWristAngle(Pose2d pose, double armDistance){
    horizontalDistance = pose.getX()+armDistance;
    return Math.atan(height/horizontalDistance);
  }

  public Translation2d getShootingArmPose(Pose2d pose, double armDistance){
    return new Translation2d(20,calculateWristAngle(pose,armDistance));
  }
  //Shooter april tag seen






}