package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private WPI_Pigeon2 gyro;
    private Rotation2d gyroOffset = new Rotation2d();

    private final SwerveModule[] swerveModules;

    private SwerveDriveKinematics kinematics;



    public SwerveDrive(WPI_Pigeon2 gyro, SwerveModule... modules) {
        this.gyro = gyro;
        this.swerveModules = modules;


        Translation2d[] translation2d = new Translation2d[swerveModules.length];
        for ( int i =0; i < swerveModules.length; i ++)    {

            SwerveModule state =  swerveModules[i];

            translation2d[i] = state.getLocation();




        }

        kinematics = new SwerveDriveKinematics(translation2d);





    }

    // ========== Driving ==========

    /**
     * Drive relative to the field
     * @param speeds Field-relative speeds to drive at
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * Drive relative to the robot.
     * @param speeds Robot-relative speeds to drive at
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);


        for ( int i =0; i < swerveModules.length; i ++)    {

          SwerveModule state =  swerveModules[i];
          state.setDesiredState(swerveModuleStates[i]);




        }



        //throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * Stop all motor movement
     */
    public void stop() {
        // Stop all modules
        for (SwerveModule module : this.swerveModules)
            module.stop();
    }

    // ========== Gyro ==========

    /**
     * Get angle relative to the field
     * @return Angle relative to the field
     * @see #resetFieldAngle()
     */
    public Rotation2d getFieldAngle() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * Reset the angle of field-to-robot to 0\deg
     * @see #resetFieldAngle(Rotation2d)
     */
    public void resetFieldAngle() {
        resetFieldAngle(new Rotation2d(0));
    }

    public void resetFieldAngle(Rotation2d currentAngle) {
        throw new UnsupportedOperationException("Not implemented");
    }

    // ========== Odometry ==========

    /**
     * Get the positions of each swerve module
     * @return Positions of each swerve module
     */
    protected SwerveModulePosition[] getPositions() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * Reset odometry back to (0, 0)
     * @see #resetOdometry(Pose2d) 
     */
    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    public void resetOdometry(Pose2d currentPose) {
        throw new UnsupportedOperationException("Not implemented");
    }

    @Override
    public void periodic() {
        super.periodic();
        // Update each of the modules
        for (SwerveModule module : swerveModules)
            module.periodic();
    }
}
