package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.SwerveModule;

/**
 * This class contains all the Subsystem objects, in addition to binding commands between inputs and subsystems.
 */
public class RobotContainerHorta {
	public final WPI_Pigeon2 navx;

	public SwerveModule swerveModule;


	public SwerveDrive driveChassis;
	public final Joystick driverJoystick = new Joystick(0);







	public RobotContainerHorta() {


		// Subsystems
		this.navx = new WPI_Pigeon2(0);
		double swerveWidth = Units.inchesToMeters(13.875);
		double swerveHeight = Units.inchesToMeters(13.875);
		var smFrontLeft = new SwerveModule(
				"Front Left",
				new Translation2d(+swerveWidth, +swerveHeight),

				10,
				31,
				45,
				11
		);
		var smFrontRight = new SwerveModule(
				"Front Right",
				new Translation2d(+swerveWidth, -swerveHeight),
				8,
				32,
				-135+45,
				9
		);
		var smBackRight = new SwerveModule(
				"Back Right",
				new Translation2d(-swerveWidth, -swerveHeight),
				20,
				33,
				-135,
				1
		);

		var smBackLeft = new SwerveModule(
				"Back Left",
				new Translation2d(-swerveWidth, +swerveHeight),
				18,
				34,
				135,
				19
		);

		driveChassis = new SwerveDrive(
				navx,
				smFrontLeft,
				smFrontRight,
				smBackRight,
				smBackLeft
		);

		// Commands
		var stopCommand = Commands.run(driveChassis::stop, driveChassis)
			.withName("Stop driving");

		// Default commands
		driveChassis.setDefaultCommand(stopCommand);







	}

	/**
	 * Get command to use in autonomous
	 * @return autonomous-period command, or null if we don't want to run any auto
	 */
	public Command getAutoCommand() {
		return null;
	}
}
