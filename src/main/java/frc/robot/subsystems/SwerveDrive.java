// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.StructArrayEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.AllianceFlip;
import frc.robot.UsefulPoints;
import frc.robot.vision.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;

public class SwerveDrive extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    SwerveModule FLModule;
    SwerveModule BLModule;
    SwerveModule FRModule;
    SwerveModule BRModule;
    WPI_Pigeon2 pigeon;
//    private final SwerveDriveOdometry odometer;
    private final double maxMetersPerSec;
    private final double maxMetersPerSecSquared;
    public SwerveDriveKinematics kDriveKinematics;
    double desiredYaw;
    boolean align = false;
    double kP, kI, kD, xykP, xykI, xykD;

    Vision vision = new Vision();

    private final ProfiledPIDController thetaController;
    private final ProfiledPIDController driveThetaController;
    private final PIDController xController,yController;
    Field2d field = new Field2d();
    SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    StructArrayLogEntry<SwerveModuleState> SwerveLogEntry;
    public SwerveDrive(SwerveModule FLModule, SwerveModule BLModule, SwerveModule FRModule, SwerveModule BRModule,
                       WPI_Pigeon2 pigeon, double maxMetersPerSec, double maxMetersPerSecSquared, double maxRPS, double maxRPS2,
                       double kP, double kI, double kD,
                       double xykP, double xykI, double xykD,
                       double thetaP, double thetaI, double thetaD) {

        this.pigeon = pigeon;
        this.maxMetersPerSec = maxMetersPerSec;

        SwerveLogEntry = StructArrayLogEntry.create(DataLogManager.getLog(), "Swerve/States", SwerveModuleState.struct);

        this.maxMetersPerSecSquared = maxMetersPerSecSquared;

        this.FLModule = FLModule;

        this.BLModule = BLModule;

        this.FRModule = FRModule;
        this.kP = kP; this.kD = kD; this.kI = kI;
        this.xykP = xykP; this.xykI = xykI; this.xykD = xykD;

        this.BRModule = BRModule;
        driveThetaController = new ProfiledPIDController(thetaP, thetaI, thetaD, new TrapezoidProfile.Constraints(maxRPS, maxRPS2));
        thetaController = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxRPS,maxRPS2));
        thetaController.enableContinuousInput(-180,180);
        driveThetaController.enableContinuousInput(-180,180);
        xController = new PIDController(xykP,xykI,xykD);
        yController = new PIDController(xykP,xykI,xykD);
        kDriveKinematics = new SwerveDriveKinematics(FRModule.moduleTranslation(), FLModule.moduleTranslation(),
                BRModule.moduleTranslation(), BLModule.moduleTranslation());
//        odometer = new SwerveDriveOdometry(kDriveKinematics, new Rotation2d(0), getModulePositions());
        align = false;
        SmartDashboard.putData("odometry", field);
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics, new Rotation2d(0), getModulePositions(), new Pose2d());
    }

    public double getDesiredYaw(){
        return desiredYaw;
    }

    public Command setInitPosition(Pose2d initPose){
        return Commands.sequence(Commands.runOnce(()->setPigeon(AllianceFlip.apply(initPose).getRotation().getDegrees())),
//		        Commands.runOnce(()->odometer.update(getRotation2d(),getModulePositions())),
                Commands.runOnce(() -> {}), //wait a cycle to reset the pigeon or everything breaks
                Commands.runOnce(() -> {}),
                Commands.runOnce(()->resetOdometry(AllianceFlip.apply(initPose))),
                Commands.runOnce(()->{})
        );
    }
    public void setDesiredYaw(double yaw){
        align = true;
        desiredYaw = yaw;
    }

    public double getYawCorrection(){
        return driveThetaController.calculate(getYaw()-desiredYaw);
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

//    public Pose2d getPose() {
//        return odometer.getPoseMeters();
//    }
    public Pose2d getEstimatedPose(){
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
//        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
        swerveDrivePoseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public Rotation2d getAngleBetweenSpeaker(Translation2d pos, Translation2d speaker) {
        Translation2d diff = pos.minus(speaker);
        return Rotation2d.fromRadians(MathUtil.angleModulus((Math.atan2(diff.getY(),diff.getX()))));
    }
    public Rotation2d getAngleBetweenSpeaker(Translation2d pos){
        return getAngleBetweenSpeaker(pos, UsefulPoints.Points.middleOfSpeaker);
    }
    public Rotation2d getAngleBetweenSpeaker(Supplier<Translation2d> pos){
        return getAngleBetweenSpeaker(pos.get(), AllianceFlip.apply(UsefulPoints.Points.middleOfSpeaker));
    }

    public List<Pose2d> getObjectPos(){
        ArrayList<Pose2d> desRobotPos = new ArrayList<>();
        var objectVal = vision.detections();
        for (int i = 0; i < objectVal.size(); i++){
            Translation2d fieldObjPos = objectVal.get(i).rotateBy(getRotation2d());
            Rotation2d desObjRot = Rotation2d.fromRadians(Math.atan2(fieldObjPos.getY(), fieldObjPos.getX()));
            desRobotPos.add(getEstimatedPose().plus(new Transform2d(fieldObjPos, desObjRot)));
        }
        return desRobotPos;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("yaw", getYaw());
		SmartDashboard.putNumber("Yaw2d",getRotation2d().getDegrees());
        SmartDashboard.putNumber("desired yaw", getDesiredYaw());
//        odometer.update(getRotation2d(), getModulePositions());
//        field.getObject("odom").setPose(odometer.getPoseMeters());
//        vision.setOdometryPosition(odometer.getPoseMeters());
        SmartDashboard.putNumber("Rotation",getEstimatedPose().getRotation().getDegrees());
        swerveDrivePoseEstimator.update(getRotation2d(), getModulePositions());
        var aprilTagVal = vision.getAprilTagPose();
        if (aprilTagVal.isPresent()) {
            SmartDashboard.putNumber("timeStamp", aprilTagVal.get().timestamp);
            var dist = getEstimatedPose().getTranslation().getDistance(aprilTagVal.get().pose.getTranslation());
            swerveDrivePoseEstimator.addVisionMeasurement(aprilTagVal.get().pose, aprilTagVal.get().timestamp,
                    VecBuilder.fill(5e-2,5e-2,10));
        }
        SmartDashboard.putNumberArray("detections", getObjectPos().stream().flatMapToDouble(
                pos -> DoubleStream.of(pos.getX(), pos.getX(), pos.getRotation().getDegrees())).toArray());
        field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
        SmartDashboard.putNumber("Posex", getEstimatedPose().getX());
        SmartDashboard.putNumber("Posey", getEstimatedPose().getY());
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
        config.setEndVelocity(endVelocityMetersPerSecond);
        config.setStartVelocity(startVelocityMetersPerSecond);
        var trajectory = TrajectoryGenerator.generateTrajectory(
                AllianceFlip.apply(start),
                AllianceFlip.apply(internalPoints),
                AllianceFlip.apply(end),
                config
        );
        SmartDashboard.putNumber("Time",trajectory.getTotalTimeSeconds());
        SmartDashboard.putNumber("trajEndRotation", trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("desiredEndRot", end.getRotation().getDegrees());
        SwerveControllerCommand trajCommand = new SwerveControllerCommand(
                trajectory,
//                vision::getRobotPosition,
		        this::getEstimatedPose,
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
        SwerveLogEntry.append(desiredStates);
        FRModule.setDesiredState(desiredStates[0]);
        FLModule.setDesiredState(desiredStates[1]);
        BRModule.setDesiredState(desiredStates[2]);
        BLModule.setDesiredState(desiredStates[3]);
    }

    public void driveAtSpeed(double xspd, double yspd, double turnspd, boolean fieldOriented, boolean red){
        if (turnspd != 0) {
            align = false;
        }
        if (align){
            turnspd = getYawCorrection();
        }
        if (xspd == 0 && yspd == 0 && turnspd == 0) stopModules();
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented){
            Rotation2d currRot = getRotation2d();
            if (red) currRot = getRotation2d().plus(Rotation2d.fromDegrees(180));
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xspd, yspd, turnspd, currRot);
        }
        else{
            chassisSpeeds = new ChassisSpeeds(xspd, yspd, turnspd);
        }
        SmartDashboard.putString("chassis speeds", chassisSpeeds.toString());
        SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        SmartDashboard.putString("module states", Arrays.toString(moduleStates));

        setModuleStates(moduleStates);
    }
    public void driveAtSpeed(double xspd, double yspd, double turnspd, boolean fieldOriented){
        driveAtSpeed(xspd, yspd, turnspd, fieldOriented, false);
    }

}