package frc.robot.subsystems;

/**
 * Interface for Generic Motors used in swerve modules.
 */
public interface GenericMotor {
	/**
	 * Sets the power of the motor
	 * @param power The power of the motor, with range from -1.0 to 1.0
	 */
	void set(double power);

	/**
	 * Gets the current power of the motor
	 * @return Current power of the motor, from -1.0 to 1.0
	 */
	double get();

	/**
	 * Sets motor velocity in RPM
	 * @param velocity The velocity in RPM to set the motor to
	 */
	void setMotorVelocity(double velocity);

	/**
	 * Gets current motor velocity in RPM
	 * @return Current velocity of the motor, in RPM
	 */
	double getMotorVelocity();

	/**
	 * Gets current motor position in Rotations
	 * @return Current position of the motor, in Rotations
	 */
	double getMotorPosition();
	void stopMotor();

	/**
	 * Sets the PID constants for the motor's built in controller
	 * @param motorP P constant
	 * @param motorI I constant
	 * @param motorD D constant
	 * @param motorFF Feed Forward constant
	 */
	void setPID(double motorP, double motorI, double motorD, double motorFF);

	/**
	 * Sets the default rotation direction of the motor
	 * @param invert The direction of the motor, true or false
	 */
	void setInvert(boolean invert);

	/**
	 * Gets the device ID of the motor
	 * @return The device ID of the motor
	 */
	int getDeviceID();

	/**
	 * Modifies the PID constants of the motor and returns itself
	 * @param motorP P constant
	 * @param motorI I constant
	 * @param motorD D constant
	 * @param motorFF Feed Forward constant
	 * @return A GenericMotor with the specified PID constants
	 */
	GenericMotor withPID(double motorP, double motorI, double motorD, double motorFF);

	/**
	 * Modifies the default rotation direction of the motor and returns itself
	 * @param invert The direction of the motor, true or false
	 * @return A GenericMotor with the specified default rotation direction
	 */
	GenericMotor withInvert(boolean invert);
}
