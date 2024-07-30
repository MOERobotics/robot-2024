// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GenericSwerveModule extends SubsystemBase {
    private final GenericMotor driveMotor;
    private final GenericMotor pivotMotor;

    private final WPI_CANCoder pivotEncoder;

    private final double pivotOffset;
    private final double encoderTicksPerMeter;
    private final double velocityConversionFactor;

    private final PIDController turningController;
    private final Translation2d moduleTran;


    public GenericSwerveModule(GenericMotor driveMotor, GenericMotor pivotMotor, int pivotEncoderID,
							   boolean driveInvert, boolean pivotInvert, double pivotOff, Translation2d moduleTran,
							   double encoderTicksPerMeter, double velocityConversionFactor,
                               double pivotP, double pivotI, double pivotD, double driveP,
                               double driveI, double driveD, double driveFF) {

        this.moduleTran = moduleTran;

        this.driveMotor = driveMotor;
        this.pivotMotor = pivotMotor;
        this.driveMotor.setInvert(driveInvert);
        this.pivotMotor.setInvert(pivotInvert);
        pivotEncoder = new WPI_CANCoder(pivotEncoderID);

        pivotOffset = pivotOff;

        this.encoderTicksPerMeter = encoderTicksPerMeter;
        this.velocityConversionFactor = velocityConversionFactor;

        turningController = new PIDController(pivotP, pivotI, pivotD);
        turningController.enableContinuousInput(-Math.PI, Math.PI);

        this.driveMotor.setPID(driveP,driveI,driveD,driveFF);
    }

    public double getDrivePosition() {
        return driveMotor.getMotorPosition()/encoderTicksPerMeter;
    }

    public double getDriveVelocity(){
        return driveMotor.getMotorVelocity()/velocityConversionFactor;
    }

    public double getPivotPosition(){
        double reading = pivotEncoder.getAbsolutePosition()+pivotOffset;
        SmartDashboard.putNumber("Power pivot Motor"+pivotMotor.getDeviceID(), pivotMotor.get());
        SmartDashboard.putNumber("pivot Motor"+pivotMotor.getDeviceID(), reading);
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
        SmartDashboard.putNumber("Velocity " + this.driveMotor.getDeviceID(), driveVelocity);
        SmartDashboard.putNumber("True Velocity" + this.driveMotor.getDeviceID(), getDriveVelocity());
        driveMotor.setMotorVelocity(driveVelocity);
        pivotMotor.set(turningController.calculate(getPivotRad(), state.angle.getRadians()));
    }

    public Translation2d moduleTranslation(){
        return moduleTran;
    }

    public void stop(){
        driveMotor.stopMotor();
        pivotMotor.stopMotor();
    }


}























