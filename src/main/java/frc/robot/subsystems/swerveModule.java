// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class swerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax pivotMotor;

    private final RelativeEncoder driveEncoder;
    private final CANCoder pivotEncoder;

    private final double pivotOffset;
    private final double encoderTicksPerMeter;
    private final double velocityConversionFactor;

    private final PIDController turningController;
    private final SparkPIDController driveController;
    private final Translation2d moduleTran;

    public swerveModule(int driveMotorID, int pivotMotorID, int pivotEncoderID,
                        boolean driveInvert, boolean pivotInvert, double pivotOff, Translation2d moduleTran,
                        double encoderTicksPerMeter, double velocityConversionFactor,
                        double pivotP, double pivotI, double pivotD, double driveP,
                        double driveI, double driveD, double driveFF) {

        this.moduleTran = moduleTran;

        driveMotor = new CANSparkMax(driveMotorID, kBrushless);
        pivotMotor = new CANSparkMax(pivotMotorID, kBrushless);

        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        pivotEncoder = new WPI_CANCoder(pivotEncoderID);

        driveMotor.setInverted(driveInvert);
        pivotMotor.setInverted(pivotInvert);

        pivotOffset = pivotOff;

        this.encoderTicksPerMeter = encoderTicksPerMeter;
        this.velocityConversionFactor = velocityConversionFactor;

        turningController = new PIDController(pivotP, pivotI, pivotD);
        turningController.enableContinuousInput(-Math.PI, Math.PI);

        driveController = driveMotor.getPIDController();
        driveController.setP(driveP);
        driveController.setI(driveI);
        driveController.setIZone(0);
        driveController.setD(driveD);
        driveController.setFF(driveFF);
        driveController.setOutputRange(-1, 1);
    }



    public double getDrivePosition() {
        return driveEncoder.getPosition()/encoderTicksPerMeter;
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity()/velocityConversionFactor;
    }

    public double getPivotPosition(){
        double reading = pivotEncoder.getAbsolutePosition()+pivotOffset;
        SmartDashboard.putNumber("pivot Motor"+pivotMotor.getDeviceId(), reading);
        if (reading < 0){
            return (reading - 180)%360 + 180;
        }
        return (reading + 180)%360 - 180;
    }

    public double getPivotRad(){
        //flip for ccw positive and convert to rad
        return Units.degreesToRadians(getPivotPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getPivotRad()));
    }
    public SwerveModulePosition getPosition(){
        return( new SwerveModulePosition(
                getDrivePosition(), new Rotation2d(getPivotRad())));
    }
    public void setDesiredState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        var driveVelocity = state.speedMetersPerSecond * velocityConversionFactor;
        SmartDashboard.putNumber("Velocity " + this.driveMotor.getDeviceId(), driveVelocity);
        driveController.setReference(driveVelocity, CANSparkMax.ControlType.kVelocity);
        pivotMotor.set(turningController.calculate(getPivotRad(), state.angle.getRadians()));
    }

    public Translation2d moduleTranslation(){
        return moduleTran;
    }

    public void stop(){
        driveMotor.set(0);
        pivotMotor.set(0);
    }


}























