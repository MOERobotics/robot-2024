// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CollectorCommands;
import frc.robot.commands.SwerveController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.setHeading;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.HeadSubsystem;

import java.awt.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class FortissiMOEContainer{
    WPI_Pigeon2 pigeon = new WPI_Pigeon2(0);
    /////////////////////////////////////////////////////////////////////////////drive subsystems
    double encoderTicksPerMeter = 6.75/12.375*1.03/1.022*39.3701;
    double velocityConversionFactor = 32.73*1.03/1.022 * Units.metersToInches(1);
    double pivotP = 8.0e-3*60;
    double pivotI = 0.0;
    double pivotD = 0.0;
    double driveP = 7.0e-5;
    double driveI = 0.0;
    double driveD = 1.0e-4;
    double driveFF = 1.76182e-4;
    double width = Units.inchesToMeters(14);
    double length = Units.inchesToMeters(14);
    double maxMPS = 174/39.3701;
    double maxRPS = Math.PI*2;
    private final SwerveModule backLeftModule = new SwerveModule(
            3,
            2,
            33,
            false,
            true,
            135,
            new Translation2d(-width, length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveModule backRightModule = new SwerveModule(
            17,
            16,
            34,
            false,
            true,
            -135,
            new Translation2d(-width, -length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveModule frontLeftModule = new SwerveModule(
            1,
            20,
            32,
            false,
            true,
            45,
            new Translation2d(width, length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveModule frontRightModule = new SwerveModule(
            19,
            18,
            31,
            false,
            true,
            -45,
            new Translation2d(width, -length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveDrive swerveSubsystem = new SwerveDrive(frontLeftModule, backLeftModule, frontRightModule, backRightModule,
            ()->pigeon.getYaw(), maxMPS, 3, 0, 0, 0);
    /////////////////////////////////////////////////////////////////////////////drive subsystems end
    /////////////////////////////////////////////////////////////////////////////arm subsystem start
    private final Arm armSubsystem = new Arm(4, 15,14, 35, 36,
            0, 0, 0, 0, 0, 0, new Rotation2d(0), new Rotation2d(0),
            0,0);

    /////////////////////////////////////////////////////////////////////////////arm susbsystem start

	//TODO: Replace 99 with correct motor IDs.
	private final HeadSubsystem headSubsystem = new HeadSubsystem(5,13,6,
			0,0,0,0,0,0,0,0,0, false);
    /////////////////////////////////////////////////////////////////////////// arm subsystem end

    private final Joystick driverJoystick = new Joystick(1); ///joystick imports
    private final Joystick functionJoystick = new Joystick(0);


    ////////////////////////////////////////////////////////////////////////////commands



    private final Command drive  = new SwerveController(swerveSubsystem,
            () -> -driverJoystick.getRawAxis(1),
            () -> -driverJoystick.getRawAxis(0),
            () -> -driverJoystick.getRawAxis(2),
            () -> driverJoystick.getRawButton(5),//Slow Bumper
            () -> driverJoystick.getRawButton(3),//Robot relative
		    0.15,6, maxMPS, maxRPS);

    // private final Command turnRobotOn = new CollectorOnOrOffCommand(headSubsystem, true);

    ////////////////////////////////////////////////////////////////////////////commands end



    public FortissiMOEContainer() {
        pigeon.reset();
        swerveSubsystem.setDefaultCommand(drive);
        // Configure the trigger bindings
        configureBindings();
        var headDownThenCollect = CollectorCommands.headDownThenCollect(headSubsystem, armSubsystem);
        var depositToAmp = CollectorCommands.setArmToAmpThenDeposit(headSubsystem, armSubsystem);

        var button1 = new Trigger(() -> functionJoystick.getRawButton(1));
        button1.onTrue(headSubsystem.runCollectorCommands(-.3));

        var button2 = new Trigger(() -> functionJoystick.getRawButton(2));
        button2.onTrue(headSubsystem.runCollectorCommands(.3));

        var button8 = new Trigger (() -> functionJoystick.getRawButton(8));
        button8.onTrue(headSubsystem.runCollectorCommands(0));

        if(!button1.getAsBoolean()&&!button2.getAsBoolean()){
            headSubsystem.stopCollector();
        }


        /*var button5 = new Trigger (() -> driverJoystick.getRawButton(5));
        button5.onTrue(new setHeading(swerveSubsystem,
                () -> driverJoystick.getRawAxis(1),
                () -> driverJoystick.getRawAxis(2),
                swerveSubsystem.getAngleBetweenSpeaker(swerveSubsystem.getPose().getTranslation()))
        ); */

    }



    private void configureBindings() {
        new JoystickButton(driverJoystick, 1).onTrue(Commands.runOnce(() -> {pigeon.setYaw(0); swerveSubsystem.setDesiredYaw(0);}));
        new JoystickButton(driverJoystick, 2).onTrue(Commands.run(()->armSubsystem.shoulderPower(.1)));
        new JoystickButton(driverJoystick, 4).onTrue(Commands.run(()->armSubsystem.wristPower(.1)));
        new JoystickButton(driverJoystick, 6).onTrue(Commands.run(()->armSubsystem.shoulderPower(-.1)));
        new JoystickButton(driverJoystick, 7).onTrue(Commands.run(()->armSubsystem.wristPower(-.1)));
    }

    public Command getAutonomousCommand() {
        return null;
        // return Autos.exampleAuto(m_drive);

    }
}













