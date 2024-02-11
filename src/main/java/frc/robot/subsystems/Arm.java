// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

    private final CANcoder shoulderEncoder;
    private final CANcoder wristEncoder;

    private final PIDController shoulderController;
    private final PIDController wristController;

    private Rotation2d extremeShoulder, extremeWrist, interShoulder, interWrist;
    private double maxSpeed, maxAccel, shoulderLength, wristLength;

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

        shoulderMotorLeft.setInverted(false);
        shoulderMotorRight.setInverted(true);
        wristMotor.setInverted(false);

        shoulderMotorRight.follow(shoulderMotorLeft, true);

        shoulderEncoder = new CANcoder(shoulderEncoderID);
        wristEncoder = new CANcoder(wristEncoderID);

        this.maxAccel = maxAccel; this.maxSpeed = maxSpeed;
        this.shoulderLength = shoulderLength; this.wristLength = wristLength;

        shoulderController = new PIDController(kPShoulder, kIShoulder, kDShoulder);
        wristController = new PIDController(kPWrist, kIWrist, kDWrist);
        extremeShoulder = criticalShoulderAngle; extremeWrist = criticalWristAngle;
        interShoulder = criticalShoulderAngle; interWrist = criticalWristAngle;
    }

    public void periodic(){
        //TODO: make the mechanism 2d object in here
        SmartDashboard.putNumber("shoulderValue", shoulderState().getDegrees());
        SmartDashboard.putNumber("wristValue", wristState().getDegrees());
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
        shoulderMotorLeft.set(power);
    }
    public void wristPower(double power){
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
        if (boundChecker.inPointyPart2O(shoulderState(), wristState())){
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
        return Rotation2d.fromDegrees(shoulderEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d wristState(){
        return Rotation2d.fromDegrees(wristEncoder.getAbsolutePosition().getValueAsDouble());
    }


}