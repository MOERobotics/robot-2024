// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindRamsete;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;



public class Arm extends SubsystemBase {


    private final CANSparkMax shoulderMotor;
    private final CANSparkMax wristMotor;

    private final CANcoder shoulderEncoder;
    private final CANcoder wristEncoder;

    private final SparkPIDController shoulderController;
    private final SparkPIDController wristController;
    public Supplier<ChassisSpeeds> desiredSpeeds;
    double shoulderLength, wristLength, shoulderInertia, wristInertia, maxShoulderSpeed,
            maxWristSpeed;

    // constructor
    public Arm(int shoulderMotorID, int wristMotorID, int shoulderEncoderID, int wristEncoderID,
               double kPShoulder, double kIShoulder, double kDShoulder, double kFFShoulder,
               double kPWrist, double kIWrist, double kDWrist, double kFFWrist,
               double shoulderLength, double wristLength, double shoulderInertia, double wristInertia,
               double maxShoulderSpeed, double maxWristSpeed) {
        shoulderMotor = new CANSparkMax(shoulderMotorID, kBrushless);
        wristMotor = new CANSparkMax(wristMotorID, kBrushless);

        shoulderEncoder = new CANcoder(shoulderEncoderID);
        wristEncoder = new CANcoder(wristEncoderID);

        shoulderController = shoulderMotor.getPIDController();
        wristController = wristMotor.getPIDController();

        shoulderController.setP(kPShoulder);
        shoulderController.setI(kIShoulder);
        shoulderController.setD(kDShoulder);
        shoulderController.setFF(kFFShoulder);

        wristController.setP(kPWrist);
        wristController.setI(kIWrist);
        wristController.setD(kDWrist);
        wristController.setFF(kFFWrist);

        this.shoulderLength = shoulderLength;
        this.shoulderInertia = shoulderInertia;
        this.wristLength = wristLength;
        this.wristInertia = wristInertia;
        this.maxShoulderSpeed = maxShoulderSpeed;
        this.maxWristSpeed = maxWristSpeed;


    }

    public void periodic(){
        pathFollow(desiredSpeeds.get());
        //TODO: make the mechanism 2d object in here
    }

    void pathFollow(ChassisSpeeds armSpeeds){
        //x is shoulder, y is wrist
        shoulderController.setReference(armSpeeds.vxMetersPerSecond, kVelocity);
        wristController.setReference(armSpeeds.vyMetersPerSecond, kVelocity);
    }

    ChassisSpeeds currArmSpeeds(){
        return new ChassisSpeeds(
                shoulderEncoder.getVelocity().getValueAsDouble(),
                wristEncoder.getVelocity().getValueAsDouble(), 0
        );
    }

    Rotation2d shoulderState(){
        return Rotation2d.fromDegrees(shoulderEncoder.getAbsolutePosition().getValueAsDouble());
    }

    Rotation2d wristState(){
        return Rotation2d.fromDegrees(wristEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public Pose2d getArmState(){
        return new Pose2d(new Translation2d(shoulderState().getRadians(), wristState().getRadians()),
                new Rotation2d(0));
    }

    public Command followPathCommand(Rotation2d shoulderAngle, Rotation2d wristAngle){
        Translation2d targetPose;
        return new PathfindRamsete(
                targetPose,
                new PathConstraints(3,3,
                        0,0),
                ()->getArmState(),
                () -> currArmSpeeds(),
                (ChassisSpeeds speeds) -> pathFollow(speeds),
                new ReplanningConfig(),
                this
        );
    }



}