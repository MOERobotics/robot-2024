// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.*;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;

public class SwerveModule extends SubsystemBase {
    private final SparkMax driveMotor;
    private final SparkMaxConfig driveMotorConfig;
    private final SparkMax pivotMotor;
    private final SparkMaxConfig pivotMotorConfig;

    private final RelativeEncoder driveEncoder;
    private final CANCoder pivotEncoder;

    private final double pivotOffset;
    private final double encoderTicksPerMeter;
    private final double velocityConversionFactor;

    private final PIDController turningController;
    private final SparkClosedLoopController driveController;
    private final Translation2d moduleTran;


    public SwerveModule(int driveMotorID, int pivotMotorID, int pivotEncoderID,
                        boolean driveInvert, boolean pivotInvert, double pivotOff, Translation2d moduleTran,
                        double encoderTicksPerMeter, double velocityConversionFactor,
                        double pivotP, double pivotI, double pivotD, double driveP,
                        double driveI, double driveD, double driveFF) {

        this.moduleTran = moduleTran;

        driveMotor = new SparkMax(driveMotorID, kBrushless);
        driveMotorConfig = new SparkMaxConfig();
        pivotMotor = new SparkMax(pivotMotorID, kBrushless);
        pivotMotorConfig = new SparkMaxConfig();

        driveMotorConfig.inverted(driveInvert).smartCurrentLimit(60).idleMode(SparkBaseConfig.IdleMode.kBrake);
        pivotMotorConfig.inverted(pivotInvert).smartCurrentLimit(60).idleMode(SparkBaseConfig.IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        pivotEncoder = new WPI_CANCoder(pivotEncoderID);


        pivotOffset = pivotOff;
        driveMotorConfig.closedLoopRampRate(.35);

        this.encoderTicksPerMeter = encoderTicksPerMeter;
        this.velocityConversionFactor = velocityConversionFactor;

        turningController = new PIDController(pivotP, pivotI, pivotD);
        turningController.enableContinuousInput(-Math.PI, Math.PI);

        driveController = driveMotor.getClosedLoopController();
        driveMotorConfig.closedLoop.pid(driveP,driveI,driveD).iZone(0).velocityFF(driveFF).outputRange(-1,1);

        driveMotor.configure(driveMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        pivotMotor.configure(pivotMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }



    public double getDrivePosition() {
        return driveEncoder.getPosition()/encoderTicksPerMeter;
    }

    public double getDriveVelocity(){
        return driveEncoder.getVelocity()/velocityConversionFactor;
    }

    public double getPivotPosition(){
        double reading = pivotEncoder.getAbsolutePosition()+pivotOffset;
        SmartDashboard.putNumber("Power pivot Motor"+pivotMotor.getDeviceId(), pivotMotor.get());
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
        SmartDashboard.putNumber("True Velocity" + this.driveMotor.getDeviceId(), getDriveVelocity());
        driveController.setReference(driveVelocity, SparkMax.ControlType.kVelocity);
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























