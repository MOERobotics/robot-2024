// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AllianceFlip;
import frc.robot.UsefulPoints;
import frc.robot.commands.ArmPathFollow;

import java.util.function.Supplier;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class Arm extends SubsystemBase {
    private final CANSparkMax shoulderMotorLeft, shoulderMotorRight;
    private final CANSparkMax wristMotor;

    private final CANCoder shoulderEncoder;
    private final CANCoder wristEncoder;

    private final RelativeEncoder shoulderRelEncoder;
    private final RelativeEncoder wristRelEncoder;

    private final PIDController shoulderController;
    private final PIDController wristController;


    private Rotation2d interShoulder, interWrist;
    private double currShoulder, currWrist;
    private double maxSpeed, maxAccel, shoulderLength, wristLength;
    private double wristOffset = 0;
    private double shoulderOffset = 90;

    public Arm(int rightShoulderMotorID, int leftShoulderMotorID, int wristMotorID, int shoulderEncoderID, int wristEncoderID,
               double kPShoulder, double kIShoulder, double kDShoulder,
               double kPWrist, double kIWrist, double kDWrist,
               double shoulderLength, double wristLength,
               Rotation2d criticalShoulderAngle, Rotation2d criticalWristAngle,
               double maxSpeed, double maxAccel) {

        shoulderMotorLeft = new CANSparkMax(leftShoulderMotorID, kBrushless);
        shoulderMotorRight = new CANSparkMax(rightShoulderMotorID, kBrushless);
        wristMotor = new CANSparkMax(wristMotorID, kBrushless);

        shoulderMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shoulderMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        wristMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        shoulderMotorLeft.setInverted(true);
        shoulderMotorRight.setInverted(false);
        wristMotor.setInverted(true);

        shoulderMotorRight.follow(shoulderMotorLeft, true);

        shoulderEncoder = new CANCoder(shoulderEncoderID);
        wristEncoder = new CANCoder(wristEncoderID);

        shoulderRelEncoder = shoulderMotorLeft.getEncoder();
        wristRelEncoder = wristMotor.getEncoder();

        this.maxAccel = maxAccel; this.maxSpeed = maxSpeed;
        this.shoulderLength = shoulderLength; this.wristLength = wristLength;

        shoulderController = new PIDController(kPShoulder, kIShoulder, kDShoulder);
        wristController = new PIDController(kPWrist, kIWrist, kDWrist);
        interShoulder = criticalShoulderAngle; interWrist = criticalWristAngle;
		setShoulderDesState(shoulderState().getDegrees());
		setWristDestState(wristState().getDegrees());
        shoulderController.reset();
        wristController.reset();

    }

    public void periodic(){
        //TODO: make the mechanism 2d object in here
        SmartDashboard.putNumber("shoulderValue", shoulderState().getDegrees());
        SmartDashboard.putNumber("wristValue", wristState().getDegrees());
        SmartDashboard.putNumber("shoulderRel", shoulderPosRel());
        SmartDashboard.putNumber("wristRel", wristPosRel());
        SmartDashboard.putNumber("desiredShould", getShoulderDesState());
        SmartDashboard.putNumber("desiredWrist", getWristDesState());
    }

    public void pathFollow(Rotation2d shoulder, Rotation2d wrist){
        //x is shoulder, y is wrist
        shoulderController.setSetpoint(shoulder.getDegrees());
        double shoulderPow = (shoulderController.calculate(shoulderState().getDegrees()));
        wristController.setSetpoint(wrist.getDegrees());
        double wristPow = (wristController.calculate(wristState().getDegrees()));
//        if (!boundChecker.inBounds(shoulder, wrist, shoulderLength, wristLength)){
//            if (boundChecker.negDerivShoulder(shoulderState(), shoulder, shoulderLength, wristLength)) shoulderPow = 0;
//            if (boundChecker.negDerivWrist(wristState(),wrist, wristLength)) wristPow = 0;
//        }
        shoulderPower(shoulderPow);
        wristPower(Math.min(wristPow, .5));
    }

    public void shoulderPower(double power){
        SmartDashboard.putNumber("shoulderpow", power);
        shoulderMotorLeft.set(power);
    }
    public void wristPower(double power){
        SmartDashboard.putNumber("wristpow", power);
        wristMotor.set(power);
    }

    public Command goToPoint(Rotation2d shoulderPos, Rotation2d wristPos) {

        SmartDashboard.putNumber("movingToPoint", shoulderPos.getDegrees());
        Rotation2d safeShoulder, safeWrist;
        if ((shoulderState().getDegrees() < interShoulder.getDegrees() && shoulderPos.getDegrees() < interShoulder.getDegrees()) ||
                (shoulderState().getDegrees() > interShoulder.getDegrees() && shoulderPos.getDegrees() > interShoulder.getDegrees())) {
            SmartDashboard.putBoolean("inside convex region 1 or 2", true);
            return new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel).withName("Arm to dest");
        }
        safeShoulder = interShoulder; safeWrist = interWrist;
        SmartDashboard.putBoolean("transition spot", true);
        return new SequentialCommandGroup(
                new ArmPathFollow(this, safeShoulder, safeWrist, maxSpeed, maxAccel).withName("Arm to safe"),
                new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel).withName("Arm safe -> dest")
        );
    }

    public Translation2d autoAim(Supplier<Pose2d> robotPos){
        double dist = AllianceFlip.apply(UsefulPoints.Points.middleOfSpeaker).getDistance(robotPos.get().getTranslation());
        return new Translation2d(0.466*dist + 112, 5.56*dist - 69.1);
    }


    public Command controlRobot2O(Rotation2d shoulderPos, Rotation2d wristPos){
        if (boundChecker.inPointyPart2O(shoulderState(), wristState())
        || boundChecker.inPointyPart2O(shoulderPos, wristPos)){
            return new SequentialCommandGroup(
                    new ArmPathFollow(this, interShoulder, interWrist, maxSpeed, maxAccel),
                    new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel)
            );
        }

        return new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel);
    }

    public void stopMotors(){
        wristPower(0); shoulderPower(0);
    }


    public Rotation2d shoulderState() {
        double deg = shoulderEncoder.getAbsolutePosition()+shoulderOffset;
        if (deg < -180) deg = deg + 360;
        if (deg > 180) deg = deg - 360;
        return Rotation2d.fromDegrees(deg);
    }

    public Rotation2d wristState(){
        double deg = wristEncoder.getAbsolutePosition() + wristOffset;
        if (deg < -180) deg = deg + 360;
        if (deg > 180) deg = deg - 360;
        return Rotation2d.fromDegrees(deg);
    }
    public double shoulderPosRel(){
        return shoulderRelEncoder.getPosition();
    }
    public double wristPosRel(){
        return wristRelEncoder.getPosition();
    }

    public void shoulderPowerController(double shoulderPow){
        shoulderPower(shoulderPow);
        setShoulderDesState(shoulderState().getDegrees());
    }
    public void wristPowerController(double wristPow){
        wristPower(wristPow);
        setWristDestState(wristState().getDegrees());
    }
    public void setShoulderDesState(double pos){
        currShoulder = pos;
    }
    public void setWristDestState(double pos){
        currWrist = pos;
    }

    public void setState(double shoulderDesState, double wristDesState){
        setShoulderDesState(shoulderDesState); setWristDestState(wristDesState);
    }
    public double getShoulderDesState(){
        return currShoulder;
    }
    public double getWristDesState(){
        return currWrist;
    }

    public void holdPos(double shoulderRel, double wristRel){
        SmartDashboard.putNumber("shoulderRelSetpt", shoulderRel);
        SmartDashboard.putNumber("wristRelSetpt", wristRel);
        wristController.setSetpoint(wristRel);
        shoulderController.setSetpoint(shoulderRel);
        wristPower(wristController.calculate(wristState().getDegrees()));
        shoulderPower(shoulderController.calculate(shoulderState().getDegrees()));
        //wristPower(0);
        //shoulderPower(0);
    }
    public Translation2d getAutoArmPosition(Supplier<Pose2d> currPose){
        //return a math function that interpolates this
        return new Translation2d(105, -31); //this is a placeholder
    }


}