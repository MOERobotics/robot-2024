// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ArmPathFollow;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class Arm extends SubsystemBase {
    private final CANSparkMax shoulderMotor;
    private final CANSparkMax wristMotor;

    private final CANcoder shoulderEncoder;
    private final CANcoder wristEncoder;

    private final PIDController shoulderController;
    private final PIDController wristController;

    private Rotation2d safeWrist, safeShoulder;
    private double maxSpeed, maxAccel;

    public Arm(int shoulderMotorID, int wristMotorID, int shoulderEncoderID, int wristEncoderID,
               double kPShoulder, double kIShoulder, double kDShoulder,
               double kPWrist, double kIWrist, double kDWrist,
               Rotation2d safeShoulderAngle, Rotation2d safeWristAngle,
               double maxSpeed, double maxAccel) {

        shoulderMotor = new CANSparkMax(shoulderMotorID, kBrushless);
        wristMotor = new CANSparkMax(wristMotorID, kBrushless);

        shoulderEncoder = new CANcoder(shoulderEncoderID);
        wristEncoder = new CANcoder(wristEncoderID);

        this.maxAccel = maxAccel; this.maxSpeed = maxSpeed;

        shoulderController = new PIDController(kPShoulder, kIShoulder, kDShoulder);
        wristController = new PIDController(kPWrist, kIWrist, kDWrist);
        safeShoulder = safeShoulderAngle; safeWrist = safeWristAngle;
    }

    public void periodic(){
        //TODO: make the mechanism 2d object in here
    }

    public void pathFollow(Rotation2d shoulder, Rotation2d wrist){
        //x is shoulder, y is wrist
        shoulderController.setSetpoint(shoulder.getDegrees());
        shoulderMotor.set(shoulderController.calculate(shoulderState().getDegrees()));
        wristController.setSetpoint(wrist.getDegrees());
        wristMotor.set(wristController.calculate(wristState().getDegrees()));
    }

    public Command goToPoint(Rotation2d shoulderPos, Rotation2d wristPos) {

        /*
        Add fun logic in here to find the good intermediary point
        */

        return new SequentialCommandGroup(
                new ArmPathFollow(this, safeShoulder, safeWrist, maxSpeed, maxAccel),
                new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel)
        );
    }


    public Rotation2d shoulderState(){
        return Rotation2d.fromDegrees(shoulderEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Rotation2d wristState(){
        return Rotation2d.fromDegrees(wristEncoder.getAbsolutePosition().getValueAsDouble());
    }


}