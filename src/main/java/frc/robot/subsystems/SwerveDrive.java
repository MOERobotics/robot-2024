// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Arrays;
import java.util.function.Supplier;

public class SwerveDrive extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    SwerveModule FLModule;
    SwerveModule BLModule;
    SwerveModule FRModule;
    SwerveModule BRModule;
    Supplier<Double> pigeon;
    private final SwerveDriveOdometry odometer;
    private final double maxMetersPerSec;
    SwerveDriveKinematics kDriveKinematics;
    double desiredYaw;
    private final PIDController drivePID;
    public SwerveDrive(SwerveModule FLModule, SwerveModule BLModule, SwerveModule FRModule, SwerveModule BRModule,
                       Supplier<Double> pigeon, double maxMetersPerSec, double kP, double kI, double kD) {

        this.pigeon = pigeon;
        this.maxMetersPerSec = maxMetersPerSec;

        this.FLModule = FLModule;

        this.BLModule = BLModule;

        this.FRModule = FRModule;

        this.BRModule = BRModule;
        drivePID = new PIDController(kP, kI, kD);
        kDriveKinematics = new SwerveDriveKinematics(FRModule.moduleTranslation(), FLModule.moduleTranslation(),
                BRModule.moduleTranslation(), BLModule.moduleTranslation());
        odometer = new SwerveDriveOdometry(kDriveKinematics, new Rotation2d(0), getModulePositions());

    }


    public double getYaw(){
        return pigeon.get();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("yaw", pigeon.get());
        odometer.update(getRotation2d(), getModulePositions());
    }

    public void stopModules() {
        FLModule.stop();
        FRModule.stop();
        BLModule.stop();
        BRModule.stop();
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] modulePositions = {
                FRModule.getPosition(), FLModule.getPosition(),
                BRModule.getPosition(), BLModule.getPosition()};
        return modulePositions;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxMetersPerSec);
        SmartDashboard.putString("FL state", desiredStates[0].toString());
        FRModule.setDesiredState(desiredStates[0]);
        FLModule.setDesiredState(desiredStates[1]);
        BRModule.setDesiredState(desiredStates[2]);
        BLModule.setDesiredState(desiredStates[3]);
    }

    public void driveAtSpeed(double xspd, double yspd, double turnspd, boolean fieldOriented){
        if (turnspd != 0){
            desiredYaw = pigeon.get();
        }
        else{
            turnspd = drivePID.calculate(pigeon.get(), desiredYaw);
        }
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented){
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xspd, yspd, turnspd, getRotation2d());
        }
        else{
            chassisSpeeds = new ChassisSpeeds(xspd, yspd, turnspd);
        }
        SmartDashboard.putString("chassis speeds", chassisSpeeds.toString());
        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        SmartDashboard.putString("module states", Arrays.toString(moduleStates));

        setModuleStates(moduleStates);
    }

}























