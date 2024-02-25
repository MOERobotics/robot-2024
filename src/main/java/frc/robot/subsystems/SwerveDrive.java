// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.WPIMathJNI;
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
import frc.robot.UsefulPoints;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

public class SwerveDrive extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    SwerveModule FLModule;
    SwerveModule BLModule;
    SwerveModule FRModule;
    SwerveModule BRModule;
    WPI_Pigeon2 pigeon;
    private final SwerveDriveOdometry odometer;
    private final double maxMetersPerSec;
    private final double maxMetersPerSecSquared;
    public SwerveDriveKinematics kDriveKinematics;
    double desiredYaw;
    boolean align = false;
    double kP, kI, kD, xykP, xykI, xykD;

    private final PIDController drivePID;
    Field2d field = new Field2d();
    public SwerveDrive(SwerveModule FLModule, SwerveModule BLModule, SwerveModule FRModule, SwerveModule BRModule,
                       WPI_Pigeon2 pigeon, double maxMetersPerSec, double maxMetersPerSecSquared, double kP, double kI, double kD,
                       double xykP, double xykI, double xykD) {

        this.pigeon = pigeon;
        this.maxMetersPerSec = maxMetersPerSec;

        this.maxMetersPerSecSquared = maxMetersPerSecSquared;

        this.FLModule = FLModule;

        this.BLModule = BLModule;

        this.FRModule = FRModule;
        this.kP = kP; this.kD = kD; this.kI = kI;
        this.xykP = xykP; this.xykI = xykI; this.xykD = xykD;

        this.BRModule = BRModule;
        drivePID = new PIDController(kP, kI, kD);
        drivePID.enableContinuousInput(-180,180);
        kDriveKinematics = new SwerveDriveKinematics(FRModule.moduleTranslation(), FLModule.moduleTranslation(),
                BRModule.moduleTranslation(), BLModule.moduleTranslation());
        odometer = new SwerveDriveOdometry(kDriveKinematics, new Rotation2d(0), getModulePositions());
        align = false;
        SmartDashboard.putData("odometry", field);
    }

    public double getDesiredYaw(){
        return desiredYaw;
    }

    public Command setInitPosition(Pose2d initPose){
		setPigeon(initPose.getRotation().getDegrees());
		odometer.update(getRotation2d(),getModulePositions());
        return Commands.runOnce(()->resetOdometry(AllianceFlip.apply(initPose)));
    }
    public void setDesiredYaw(double yaw){
        desiredYaw = yaw;
        headingCorrect(true);
    }
    public void headingCorrect(boolean correct){
        align = correct;
    }
    public double getYawCorrection(){
        return drivePID.calculate(getYaw()-desiredYaw);
    }

    public double getYaw(){
        return pigeon.getYaw();
    }

	public void setPigeon(double Yaw){
		pigeon.setYaw(Yaw);
	}

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(MathUtil.inputModulus(getYaw(),-180,180));
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public double getAngleBetweenSpeaker(Translation2d pos) {
        Translation2d speaker = AllianceFlip.apply(UsefulPoints.Points.middleOfSpeaker);
        Translation2d diff = pos.minus(speaker);
        return (MathUtil.angleModulus((Math.atan2(diff.getY(),diff.getX())-3*Math.PI/2)));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("yaw", getYaw());
        SmartDashboard.putNumber("desired yaw", getDesiredYaw());
        odometer.update(getRotation2d(), getModulePositions());
        field.setRobotPose(odometer.getPoseMeters());
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


    public Command generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints) {
        return generateTrajectory(start, end, internalPoints, 0,0);
    }

    public Command generateTrajectory(Pose2d start, Pose2d end, ArrayList<Translation2d> internalPoints, double startVelocityMetersPerSecond, double endVelocityMetersPerSecond){
        TrajectoryConfig config = new TrajectoryConfig(maxMetersPerSec,maxMetersPerSecSquared);
        PIDController xController = new PIDController(xykP,xykI,xykD);
        PIDController yController = new PIDController(xykP,xykI,xykD);
        var thetaController = new ProfiledPIDController(kP,kI,kD,new TrapezoidProfile.Constraints(maxMetersPerSec,maxMetersPerSecSquared));
        thetaController.enableContinuousInput(-180,180);
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
        return Commands.parallel(
                Commands.runOnce(() -> field.getObject("traj").setTrajectory(trajectory)),
                trajCommand
        );
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
        if (align){
            turnspd = drivePID.calculate(pigeon.getYaw(), desiredYaw);
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