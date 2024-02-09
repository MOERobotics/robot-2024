// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.AllianceFlip;

import java.util.ArrayList;
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
    private final double maxMetersPerSecSquared;
    SwerveDriveKinematics kDriveKinematics;
    double desiredYaw;
    private final PIDController drivePID;
    public SwerveDrive(SwerveModule FLModule, SwerveModule BLModule, SwerveModule FRModule, SwerveModule BRModule,
                       Supplier<Double> pigeon, double maxMetersPerSec, double maxMetersPerSecSquared, double kP, double kI, double kD) {

        this.pigeon = pigeon;
        this.maxMetersPerSec = maxMetersPerSec;

        this.maxMetersPerSecSquared = maxMetersPerSecSquared;

        this.FLModule = FLModule;

        this.BLModule = BLModule;

        this.FRModule = FRModule;

        this.BRModule = BRModule;
        drivePID = new PIDController(kP, kI, kD);
        kDriveKinematics = new SwerveDriveKinematics(FRModule.moduleTranslation(), FLModule.moduleTranslation(),
                BRModule.moduleTranslation(), BLModule.moduleTranslation());
        odometer = new SwerveDriveOdometry(kDriveKinematics, new Rotation2d(0), getModulePositions());

    }

	public Command setInitPosition(Pose2d initPose){
		return Commands.runOnce(()->resetOdometry(AllianceFlip.apply(initPose)));
	}

    public double getYaw(){
        return pigeon.get();
    }
    public void setDesiredYaw(double yaw) {
        desiredYaw = yaw;
    }

    public double getDesiredYaw(){
        return desiredYaw;
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

	public double getAngleBetweenSpeaker(Translation2d pos) {
		Translation2d speaker = AllianceFlip.apply(new Translation2d(0, Units.inchesToMeters(219)));
		Translation2d diff = pos.minus(speaker);
		return (MathUtil.angleModulus(Math.atan2(diff.getY(),diff.getX())+Math.PI));
	}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("yaw", pigeon.get());
        SmartDashboard.putNumber("desired yaw", getDesiredYaw());
        odometer.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Posex",getPose().getX());
        SmartDashboard.putNumber("Posey",getPose().getY());
        SmartDashboard.putNumber("Rotation",getPose().getRotation().getDegrees());
    }

    public void stopModules() {
        FLModule.stop();
        FRModule.stop();
        BLModule.stop();
        BRModule.stop();
    }


	public SwerveControllerCommand generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints) {
		return generateTrajectory(start, end, internalPoints, 0,0);
	}

    public SwerveControllerCommand generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints, double startVelocityMetersPerSecond, double endVelocityMetersPerSecond){
        TrajectoryConfig config = new TrajectoryConfig(maxMetersPerSec,maxMetersPerSecSquared);
        PIDController xController = new PIDController(0.04,0,0);
        PIDController yController = new PIDController(0.04,0,0);
        var thetaController = new ProfiledPIDController(0.1,0,0,new TrapezoidProfile.Constraints(Math.PI,Math.PI));
        config.setEndVelocity(endVelocityMetersPerSecond);
        config.setStartVelocity(startVelocityMetersPerSecond);
        var trajectory = TrajectoryGenerator.generateTrajectory(
                AllianceFlip.apply(start),
                AllianceFlip.apply(internalPoints),
		        AllianceFlip.apply(end),
                config
        );
        SmartDashboard.putNumber("Time",trajectory.getTotalTimeSeconds());
        SwerveControllerCommand trajCommand = new SwerveControllerCommand(
                        trajectory,
                        this::getPose,
                        kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        this::setModuleStates,
                        this
        );
        return trajCommand;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] modulePositions = {
                FRModule.getPosition(), FLModule.getPosition(),
                BRModule.getPosition(), BLModule.getPosition()};
        return modulePositions;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxMetersPerSec);
        SmartDashboard.putString("FL state", desiredStates[1].toString());
        SmartDashboard.putString("FR state", desiredStates[0].toString());
        SmartDashboard.putString("BR state", desiredStates[2].toString());
        SmartDashboard.putString("BL state", desiredStates[3].toString());
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
            //turnspd = drivePID.calculate(pigeon.get(), desiredYaw);
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























