// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmPathFollow;

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

    private final PIDController shoulderRelController, wristRelController;

    private Rotation2d extremeShoulder, extremeWrist, interShoulder, interWrist;
    private double currShoulder, currWrist;
    private double maxSpeed, maxAccel, shoulderLength, wristLength;

    public Arm(int rightShoulderMotorID, int leftShoulderMotorID, int wristMotorID, int shoulderEncoderID, int wristEncoderID,
               double kPShoulder, double kIShoulder, double kDShoulder,
               double kPWrist, double kIWrist, double kDWrist,
               double kP, double kI, double kD,
               double shoulderLength, double wristLength,
               Rotation2d criticalShoulderAngle, Rotation2d criticalWristAngle,
               double maxSpeed, double maxAccel) {

        shoulderMotorLeft = new CANSparkMax(leftShoulderMotorID, kBrushless);
        shoulderMotorRight = new CANSparkMax(rightShoulderMotorID, kBrushless);
        wristMotor = new CANSparkMax(wristMotorID, kBrushless);

        shoulderMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shoulderMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        wristMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        shoulderMotorLeft.setInverted(false);
        shoulderMotorRight.setInverted(true);
        wristMotor.setInverted(false);

        shoulderMotorRight.follow(shoulderMotorLeft, true);

        shoulderEncoder = new CANCoder(shoulderEncoderID);
        wristEncoder = new CANCoder(wristEncoderID);

        shoulderRelEncoder = shoulderMotorLeft.getEncoder();
        wristRelEncoder = wristMotor.getEncoder();

        this.maxAccel = maxAccel; this.maxSpeed = maxSpeed;
        this.shoulderLength = shoulderLength; this.wristLength = wristLength;

        shoulderController = new PIDController(kPShoulder, kIShoulder, kDShoulder);
        wristController = new PIDController(kPWrist, kIWrist, kDWrist);
        shoulderRelController = new PIDController(kP, kI, kD);
        wristRelController = new PIDController(kP, kI, kD);
        extremeShoulder = criticalShoulderAngle; extremeWrist = criticalWristAngle;
        interShoulder = criticalShoulderAngle; interWrist = criticalWristAngle;
		setShoulderDesState(shoulderState().getDegrees());
		setWristDestState(wristState().getDegrees());
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
        if (!boundChecker.inBounds(shoulder, wrist, shoulderLength, wristLength)){
            if (boundChecker.negDerivShoulder(shoulderState(), shoulder, shoulderLength, wristLength)) shoulderPow = 0;
            if (boundChecker.negDerivWrist(wristState(),wrist, wristLength)) wristPow = 0;
        }
        shoulderPower(shoulderPow);
        wristPower(wristPow);
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

        Rotation2d safeShoulder, safeWrist;
        if (wristState().getDegrees() < extremeWrist.getDegrees() && wristPos.getDegrees() < extremeWrist.getDegrees() ||
        wristState().getDegrees() > extremeWrist.getDegrees() && wristPos.getDegrees() < extremeWrist.getDegrees()){
            return new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel);
        }
        safeShoulder = Rotation2d.fromDegrees((wristPos.getDegrees()-wristState().getDegrees())/
                (extremeWrist.getDegrees()-wristPos.getDegrees())
                *(extremeShoulder.getDegrees()-shoulderPos.getDegrees()));
        if (safeShoulder.getDegrees() >= extremeShoulder.getDegrees()){
            return new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel);
        }
        safeShoulder = extremeShoulder; safeWrist = extremeWrist;
        return new SequentialCommandGroup(
                new ArmPathFollow(this, safeShoulder, safeWrist, maxSpeed, maxAccel),
                new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel)
        );
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


    public Rotation2d shoulderState(){
        return Rotation2d.fromDegrees(shoulderEncoder.getAbsolutePosition()+90);
    }

    public Rotation2d wristState(){
        return Rotation2d.fromDegrees(wristEncoder.getAbsolutePosition());
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
    public double getShoulderDesState(){
        return currShoulder;
    }
    public double getWristDesState(){
        return currWrist;
    }

    public void holdPos(double shoulderRel, double wristRel){
        SmartDashboard.putNumber("shoulderRelSetpt", shoulderRel);
        SmartDashboard.putNumber("wristRelSetpt", wristRel);
        wristRelController.setSetpoint(wristRel);
        shoulderRelController.setSetpoint(shoulderRel);
        wristPower(wristRelController.calculate(wristState().getDegrees()));
        shoulderPower(shoulderRelController.calculate(shoulderState().getDegrees()));
    }


}