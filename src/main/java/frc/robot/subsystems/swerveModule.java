package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;

public class SwerveModule {
    private Translation2d location;

    // Pivot motor
    private final String name;
    private final CANSparkMax pivotMotor;
    private final PIDController pivotPID = new PIDController(8e-3, 0, 0);
    private final RelativeEncoder pivotRelativeEncoder;
    private final WPI_CANCoder pivotAbsoluteEncoder;
    private final double pivotAbsoluteOffset;

    // Drive motor
    private final CANSparkMax driveMotor;
    private final SparkPIDController drivePID;
    private final RelativeEncoder driveRelativeEncoder;
    private double driveStartDistance = 0;



    public SwerveModule(String name, Translation2d location, CANSparkMax pivotMotor, WPI_CANCoder pivotAbsoluteEncoder, double pivotAbsoluteOffset, CANSparkMax driveMotor) {
        this.name = Objects.requireNonNull(name, "Swerve module name");
        this.location = Objects.requireNonNull(location, "Swerve module location");


        // Configure pivot motor
        this.pivotMotor = pivotMotor;
        pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.pivotRelativeEncoder = pivotMotor.getEncoder();

        this.pivotAbsoluteEncoder = pivotAbsoluteEncoder;
        this.pivotAbsoluteOffset = pivotAbsoluteOffset;

        // Configure PID input to handle angle wrapping
        this.pivotPID.enableContinuousInput(-180, 180);
        // Tell SmartDashboard that our pivot is aligned if we have <4° of error
        this.pivotPID.setTolerance(4);

        // Configure drive motor
        this.driveMotor = driveMotor;
        driveMotor.setInverted(false);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.driveRelativeEncoder = driveMotor.getEncoder();

        // Configure PID
        this.drivePID = driveMotor.getPIDController();
        drivePID.setP(7.0e-5);
        drivePID.setI(0);
        drivePID.setIZone(0);
        drivePID.setD(1.0e-4);
        drivePID.setFF(1.76182e-4);
        drivePID.setOutputRange(-1,1);


        Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    }

    public SwerveModule(String name, Translation2d location, int pivotMotorId, int pivotAbsoluteEncoderId, double pivotAbsoluteOffset, int driveMotorId) {
        this(name, location, new CANSparkMax(pivotMotorId, CANSparkLowLevel.MotorType.kBrushless), new WPI_CANCoder(pivotAbsoluteEncoderId), pivotAbsoluteOffset, new CANSparkMax(driveMotorId, CANSparkLowLevel.MotorType.kBrushless));
    }

    public SwerveModule(String name, double x, double y, int pivotMotorId, int pivotAbsoluteEncoderId, double pivotAbsoluteOffset, int driveMotorId) {
        this(name, new Translation2d(x, y), new CANSparkMax(pivotMotorId, CANSparkLowLevel.MotorType.kBrushless), new WPI_CANCoder(pivotAbsoluteEncoderId), pivotAbsoluteOffset, new CANSparkMax(driveMotorId, CANSparkLowLevel.MotorType.kBrushless));
    }

    /**
     * Get the location of this module within the swerve drive
     * @return Location within swerve drive (units in meters)
     */
    public Translation2d getLocation() {


        return this.location;





    }

    /**
     * We use Rotation2d in this method to remove confusion between radians and degrees
     * @return The current angle that the pivot motor is at
     */
    protected Rotation2d getPivotAngle() {

        return Rotation2d.fromDegrees(pivotAbsoluteEncoder.getPosition() + pivotAbsoluteOffset);
    }

    /**
     * Drive the pivot motor towards an angle
     * @param angle Angle to turn towards
     */
    private void setDesiredPivotAngle(Rotation2d angle) {

        double pid = pivotPID.calculate(getPivotAngle().getDegrees(), angle.getDegrees());
        pivotMotor.set(pid);

    }

    public void resetPivotPID() {
        pivotPID.reset();
    }

    /**
     * Convert a speed (in meters/second) to the equivalent RPM that the drive motor
     * would have to turn
     * @param metersPerSecond Speed (in meters/second)
     * @return Speed (RPM)
     */
    private double convertDriveMpsToRpm(double metersPerSecond) {
        final var RPM_PER_IPS = 32.73*1.03/1.022;
        return RPM_PER_IPS * Units.metersToInches(metersPerSecond);
}

    /**
     * Drive the drive motor at a given speed
     * @param speedMetersPerSecond Speed to drive at, in m/s
     */
    private void setDesiredSpeed(double speedMetersPerSecond) {
        double speedrpm = convertDriveMpsToRpm(speedMetersPerSecond);


        drivePID.setReference(speedrpm, CANSparkMax.ControlType.kVelocity);

    }

    /**
     * Convert a number of encoder ticks to its equivalent distance in meters
     * @param ticks Number of encoder ticks
     * @return Distance (meters)
     */
    private double convertDriveTicksToMeters(double ticks) {


        var ticksPerInch = 6.75/12.375*1.03/1.022;
        var inches = ticks * ticksPerInch;
        return Units.inchesToMeters(inches);
    }

    /**
     * Get the distance travelled by the drive motor, in ticks
     * @return Distance (ticks)
     */
    private double getDistanceTicks(){

        return pivotAbsoluteEncoder.getAbsolutePosition();
    }

    /**
     * Get the distance travelled by the drive motor, in meters
     *
     * @return Distance (meters)
     * @see #getDistanceTicks()
     */
    public double getDistanceMeters() {


        return convertDriveTicksToMeters(getDistanceTicks());
    }

    /**
     * Set the desired state for this module.
     *
     * This will drive the module towards an angle and RPM.
     *
     * @param state
     */
    public void setDesiredState(SwerveModuleState state) {


        setDesiredSpeed(state.speedMetersPerSecond);
        setDesiredPivotAngle(state.angle);

    }

    /**
     * Get the current position of this swerve module.
     *
     * This method is useful for odometry
     * @return This module's position
     */
    public SwerveModulePosition getPosition() {
        throw new UnsupportedOperationException("Not implemented");
    }

    /**
     * Stop moving
     */
    public void stop() {
        this.pivotMotor.stopMotor();
        this.driveMotor.stopMotor();
    }

    public void periodic() {
        // Debug some useful values to SmartDashboard
        SmartDashboard.putNumber(name + " pivot angle", getPivotAngle().getDegrees());
        SmartDashboard.putBoolean(name + " pivot aligned", pivotPID.atSetpoint());
        SmartDashboard.putNumber(name + " drive (ticks)", getDistanceTicks());
        SmartDashboard.putNumber(name + " drive (meters)", getDistanceMeters());
        SmartDashboard.putNumber(name + " drive velocity (RPM)", driveRelativeEncoder.getVelocity());
    }
}
