// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.AutoCodeLines;

import org.opencv.core.Point;
import java.util.ArrayList;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class Arm extends SubsystemBase {
    private final CANSparkMax shoulderMotor;
    private final CANSparkMax wristMotor;

    private final CANcoder shoulderEncoder;
    private final CANcoder wristEncoder;

    private final PIDController shoulderController;
    private final PIDController wristController;

    private Trajectory trajectory;
    private Timer timer;
    private Point intermediaryPoint;
    private boolean finSpot = false;
    private boolean armInPos = false;
    private Point desPose, startPose;

    double shoulderLength, wristLength, shoulderInertia, wristInertia, maxArmSpeed, targetDist;

    public Arm(int shoulderMotorID, int wristMotorID, int shoulderEncoderID, int wristEncoderID,
               double kPShoulder, double kIShoulder, double kDShoulder,
               double kPWrist, double kIWrist, double kDWrist,
               double maxArmSpeed, Rotation2d safeShoulderAngle, Rotation2d safeWristAngle) {
        shoulderMotor = new CANSparkMax(shoulderMotorID, kBrushless);
        wristMotor = new CANSparkMax(wristMotorID, kBrushless);

        shoulderEncoder = new CANcoder(shoulderEncoderID);
        wristEncoder = new CANcoder(wristEncoderID);


        shoulderController = new PIDController(kPShoulder, kIShoulder, kDShoulder);
        wristController = new PIDController(kPWrist, kIWrist, kDWrist);

        this.maxArmSpeed = maxArmSpeed;

        intermediaryPoint = new Point(safeShoulderAngle.getDegrees(), safeWristAngle.getDegrees());
        finSpot = true;
        desPose = getArmState();
        timer = new Timer();
        armInPos = false;
    }

    public void periodic(){
        //TODO: make the mechanism 2d object in here
        wayPointFollow(timer.get());
    }

    void pathFollow(Point desState){
        //x is shoulder, y is wrist
        shoulderController.setSetpoint(desState.x);
        shoulderMotor.set(shoulderController.calculate(shoulderState().getRadians()));
        wristController.setSetpoint(desState.y);
        wristMotor.set(wristController.calculate(wristState().getRadians()));
    }


    Rotation2d shoulderState(){
        return Rotation2d.fromDegrees(shoulderEncoder.getAbsolutePosition().getValueAsDouble());
    }

    Rotation2d wristState(){
        return Rotation2d.fromDegrees(wristEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Point getArmState(){
        return new Point(shoulderState().getRadians(), wristState().getRadians());
    }

    public void goToPoint(Rotation2d shoulderPose, Rotation2d wristPose){
        desPose = new Point(shoulderPose.getDegrees(), shoulderPose.getDegrees());
        startPose = getArmState();
        double xDel = startPose.x-intermediaryPoint.x;
        double yDel = startPose.y-intermediaryPoint.y;
        targetDist = Math.sqrt(xDel*xDel+yDel*yDel);
        finSpot = false;
        armInPos = false;
    }

    public void wayPointFollow(double time){

        double s = AutoCodeLines.getS(targetDist,.2, maxArmSpeed, time);
        double shoulderPos, wristPos;

        if (!finSpot){
            shoulderPos = AutoCodeLines.getPositionX(startPose, intermediaryPoint, s);
            wristPos = AutoCodeLines.getPositionY(startPose, intermediaryPoint, s);
            double deltX = (intermediaryPoint.x-getArmState().x);
            double deltY = (intermediaryPoint.y-getArmState().y);
            double delt = Math.sqrt(deltY*deltY+deltX*deltX);
            if (delt <= .1){
                timer.restart();
                finSpot = true;
                double xDel = desPose.x-getArmState().x;
                double yDel = desPose.y-getArmState().y;
                targetDist = Math.sqrt(xDel*xDel+yDel*yDel);
            }
        }
        else{
            shoulderPos = AutoCodeLines.getPositionX(intermediaryPoint, desPose, s);
            wristPos = AutoCodeLines.getPositionY(intermediaryPoint, desPose, s);
            double deltX = (desPose.x-getArmState().x);
            double deltY = (desPose.y-getArmState().y);
            double delt = Math.sqrt(deltY*deltY+deltX*deltX);
            if (delt <= .1){
                shoulderPos = desPose.x; wristPos = desPose.y;
                armInPos = true;
            }
        }
        pathFollow(new Point(shoulderPos, wristPos));
    }

    public boolean armInPos(){
        return this.armInPos;
    }





}