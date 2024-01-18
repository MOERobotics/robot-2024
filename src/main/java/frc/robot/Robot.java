/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
	private Command autoCommand;

	private RobotContainerHorta robotContainer;

	@Override
	public void robotInit() {
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard.
		System.out.println("Klaatu barada nikto.");
		SmartDashboard.putString("Status: ","Klaatu barada nikto");

		// Starts recording to data log
		DataLogManager.start();
		// Record both DS control and joystick data
		DriverStation.startDataLog(DataLogManager.getLog());

		try {
			this.robotContainer = new RobotContainerHorta();
		} catch (Exception e) {
			e.printStackTrace(System.out);
			throw e;
		}
	}


	@Override
	public void simulationPeriodic() {
		// Stop the simulation warning
	}

	@Override
	public void robotPeriodic() {
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * Stop the currently-scheduled auto command (if it's running)
	 */
	private void stopAuto() {
		Command autoCommand = this.autoCommand;
		this.autoCommand = null;
		if (autoCommand != null)
			autoCommand.cancel();
	}

	@Override
	public void autonomousInit() {
		// Stop the old auto command (if we run auto twice in testing)
		this.stopAuto();

		// Get the new auto command and schedule it
		this.autoCommand = robotContainer.getAutoCommand();
		if (this.autoCommand != null)
			this.autoCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running.
		this.stopAuto();
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}
}
