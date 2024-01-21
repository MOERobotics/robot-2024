// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class Arm extends SubsystemBase {
    private final CANSparkMax shoulderMotor;
    private final CANSparkMax wristMotor;

    private final CANcoder shoulderEncoder;
    private final CANcoder wristEncoder;

    private final PIDController shoulderController;
    private final PIDController wristController;
    double shoulderLength, wristLength, shoulderInertia, wristInertia;

    public Arm(int shoulderMotorID, int wristMotorID, int shoulderEncoderID, int wristEncoderID,
               double kPShoulder, double kIShoulder, double kDShoulder,
               double kPWrist, double kIWrist, double kDWrist,
               double shoulderLength, double wristLength, double shoulderInertia, double wristInertia) {
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
    }

    public void periodic(){
        //TODO: make the mechanism 2d object in here
    }

    void setToState(State desState){
        shoulderController.setSetpoint(desState.shoulderAngle.getRadians());
        shoulderMotor.set(shoulderController.calculate(shoulderState().getRadians()));

        wristController.setSetpoint(desState.wristAngle.getRadians());
        wristMotor.set(wristController.calculate(wristState().getRadians()));
    }

    Rotation2d shoulderState(){
        return Rotation2d.fromDegrees(shoulderEncoder.getAbsolutePosition().getValueAsDouble());
    }

    Rotation2d wristState(){
        return Rotation2d.fromDegrees(wristEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public State armInverseKinematics(double x, double y){
        State ans = new State();
        double sqLenShoulder = shoulderLength*shoulderLength;
        double sqLenWrist = wristLength*wristLength;
        double desLen = x*x + y*y;
        ans.shoulderAngle = Rotation2d.fromDegrees(Math.acos((sqLenWrist-sqLenShoulder-desLen)/(-2*shoulderLength*Math.sqrt(desLen))) +
                Math.PI*2 - Math.atan2(y,x));
        ans.wristAngle = Rotation2d.fromDegrees(Math.acos((desLen-sqLenWrist-sqLenShoulder)/(-2*shoulderLength*wristLength)));
        return ans;
    }

    public State getArmState(){
        State ans = new State();
        ans.shoulderAngle = shoulderState();
        ans.wristAngle = wristState();
        return ans;
    }

    public static class State {
        Rotation2d shoulderAngle;
        Rotation2d wristAngle;
    }
}