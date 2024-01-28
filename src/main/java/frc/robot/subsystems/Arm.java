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

    double shoulderLength, wristLength, shoulderInertia, wristInertia, maxShoulderSpeed,
    maxWristSpeed;

    // constructor
    public Arm(int shoulderMotorID, int wristMotorID, int shoulderEncoderID, int wristEncoderID,
               double kPShoulder, double kIShoulder, double kDShoulder,
               double kPWrist, double kIWrist, double kDWrist,
               double shoulderLength, double wristLength, double shoulderInertia, double wristInertia,
               double maxShoulderSpeed, double maxWristSpeed) {
        shoulderMotor = new CANSparkMax(shoulderMotorID, kBrushless);
        wristMotor = new CANSparkMax(wristMotorID, kBrushless);

        shoulderEncoder = new CANcoder(shoulderEncoderID);
        wristEncoder = new CANcoder(wristEncoderID);


        shoulderController = new PIDController(kPShoulder, kIShoulder, kDShoulder);
        wristController = new PIDController(kPWrist, kIWrist, kDWrist);

        this.shoulderLength = shoulderLength;
        this.shoulderInertia = shoulderInertia;
        this.wristLength = wristLength;
        this.wristInertia = wristInertia;
        this.maxShoulderSpeed = maxShoulderSpeed;
        this.maxWristSpeed = maxWristSpeed;

        timer = new Timer();

    }

    public void periodic(){
        //TODO: make the mechanism 2d object in here
        pathFollow(trajectory.sample(timer.get()).poseMeters);
    }

    void pathFollow(Pose2d desState){
        //x is shoulder, y is wrist
        shoulderController.setSetpoint(desState.getX());
        shoulderMotor.set(shoulderController.calculate(shoulderState().getRadians()));
        wristController.setSetpoint(desState.getY());
        wristMotor.set(wristController.calculate(wristState().getRadians()));
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

    public void goToPoint(Pose2d targetDest){
        var startPose = getArmState();

        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d((targetDest.getX()+startPose.getX())/3,
                (targetDest.getY()+startPose.getY())/3));
        interiorWaypoints.add(new Translation2d(2*(targetDest.getX()+startPose.getX())/3,
                2*(targetDest.getY()+startPose.getY())/3));

        TrajectoryConfig config = new TrajectoryConfig(maxShoulderSpeed, maxWristSpeed);
        config.setReversed(true);

        trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,
                interiorWaypoints,
                targetDest,
                config);
        timer.restart();
    }





}