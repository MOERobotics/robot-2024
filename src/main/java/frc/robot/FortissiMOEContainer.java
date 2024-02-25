// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import frc.robot.commands.autos.doubleNoteAutos;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CollectorControllerCommand;
import frc.robot.commands.ShooterControllerCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveController;
import frc.robot.subsystems.*;

import java.util.Set;


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
    double maxRPS = Math.PI;

    double maxMPSSquared = 6;
    private final SwerveModule frontRightModule = new SwerveModule(
            3,
            2,
            33,
            false,
            true,
            -45,
            new Translation2d(-width, length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveModule frontLeftModule = new SwerveModule(
            17,
            16,
            34,
            false,
            true,
            45,
            new Translation2d(-width, -length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveModule backRightModule = new SwerveModule(
            1,
            20,
            32,
            false,
            true,
            -135,
            new Translation2d(width, length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveModule backLeftModule = new SwerveModule(
            19,
            18,
            31,
            false,
            true,
            135,
            new Translation2d(width, -length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveDrive swerveSubsystem = new SwerveDrive(frontLeftModule, backLeftModule, frontRightModule, backRightModule,
            pigeon, maxMPS, maxMPSSquared,0.04, 0, 0, .01, 0, 0,1.6,0,0);
    /////////////////////////////////////////////////////////////////////////////drive subsystems end
    /////////////////////////////////////////////////////////////////////////////arm subsystem start
    private final Arm armSubsystem = new Arm(4, 15,14, 35, 36,
            1.0e-2, 1.0e-3, 0, 3.0e-2, 3.0e-3, 0, 1.0e-2,1.0e-3,0,0, 0, Rotation2d.fromDegrees(103),
            Rotation2d.fromDegrees(-53), 30,30);

    /////////////////////////////////////////////////////////////////////////// arm subsystem end

    ///////////////////////////////////////////////////////////////////////////////////////head subsystem
	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(5,
            13,1e-4, 0,0,driveFF);
    private final CollectorSubsystem collectorSubsystem = new CollectorSubsystem(6,
            0.01,0,0,0,7);
    ///////////////////////////////////////////////////////////////////////////////////////head subsystem


    private final Joystick driverJoystick = new Joystick(1); ///joystick imports
	private final Joystick functionJoystick = new Joystick(0);


    ////////////////////////////////////////////////////////////////////////////commands



    private final Command drive  = new SwerveController(swerveSubsystem,
            () -> -driverJoystick.getRawAxis(1),
            () -> -driverJoystick.getRawAxis(0),
            () -> driverJoystick.getRawAxis(2),
            () -> driverJoystick.getRawButton(5),
            () -> driverJoystick.getRawButton(3), 2,2, maxMPS, maxRPS
    );

    // private final Command turnRobotOn = new CollectorOnOrOffCommand(headSubsystem, true);
    Command collectorCommand = new CollectorControllerCommand(
            0.6,
            ()->functionJoystick.getRawAxis(2)>=0.5,
            ()->functionJoystick.getRawAxis(3)>=0.5,
            ()->functionJoystick.getRawButton(6),
            collectorSubsystem
    );
    ////////////////////////////////////////////////////////////////////////////commands end

    Command shooterControl = new ShooterControllerCommand(shooterSubsystem, armSubsystem::getShoulderDesState,
            ()->functionJoystick.getRawButtonPressed(5));
    ////////////////////////////////////////////////////////////////////////////commands end



    public FortissiMOEContainer() {
        shooterSubsystem.setShooterRPMTolerance(500);
        swerveSubsystem.setDefaultCommand(drive);
        //collectorSubsystem.setDefaultCommand(collectorCommand);
        armSubsystem.setDefaultCommand(Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(),
                armSubsystem.getWristDesState()), armSubsystem));
        //armSubsystem.setDefaultCommand(Commands.run(()->armSubsystem.stopMotors(), armSubsystem));
        // Configure the trigger bindings
        configureBindings();
//        var headDownThenCollect = CollectorCommands.headDownThenCollect(headSubsystem, armSubsystem);
//        var depositToAmp = CollectorCommands.setArmToAmpThenDeposit(headSubsystem, armSubsystem);


        collectorSubsystem.setDefaultCommand(collectorCommand);

        shooterSubsystem.setDefaultCommand(shooterControl);
	    // Configure the trigger bindings
	    configureBindings();

        SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
//

    }



    private void configureBindings() {
        new JoystickButton(driverJoystick, 1).onTrue(Commands.runOnce(() -> {pigeon.setYaw(0); swerveSubsystem.setDesiredYaw(0);}));
        new JoystickButton(functionJoystick, 8).whileTrue(Commands.run(()->armSubsystem.shoulderPowerController(.1)));
        //new JoystickButton(functionJoystick, 1).whileTrue(Commands.run(()->armSubsystem.wristPowerController(.1)));
        new JoystickButton(functionJoystick, 7).whileTrue(Commands.run(()->armSubsystem.shoulderPowerController(-.1)));
        //new JoystickButton(functionJoystick, 2).whileTrue(Commands.run(()->armSubsystem.wristPowerController(-.1)));
        new JoystickButton(functionJoystick, 1).onTrue(Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(79), Rotation2d.fromDegrees(-41)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(4) || functionJoystick.getRawButton(8) ||
                        functionJoystick.getRawButton(9))));
        new JoystickButton(functionJoystick, 2).onTrue(Commands.defer(() -> armSubsystem.goToPoint(Rotation2d.fromDegrees(105), Rotation2d.fromDegrees(-46.5)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(4) || functionJoystick.getRawButton(1) ||
                        functionJoystick.getRawButton(8))));
        new JoystickButton(functionJoystick, 4).onTrue(Commands.defer(() ->armSubsystem.goToPoint(Rotation2d.fromDegrees(104), Rotation2d.fromDegrees(-37)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(2) || functionJoystick.getRawButton(8) ||
                        functionJoystick.getRawButton(1))));
        new JoystickButton(functionJoystick, 3).onTrue(Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(132.6), Rotation2d.fromDegrees(-150)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(2) || functionJoystick.getRawButton(8) ||
                        functionJoystick.getRawButton(1) || functionJoystick.getRawButton(4))));
        //104,-41

    }

    public Command getAutonomousCommand() {
        return new doubleNoteAutos(swerveSubsystem,armSubsystem,shooterSubsystem,collectorSubsystem,0,0).DoubleNoteAuto1();
        // return Autos.exampleAuto(m_drive);
    }
    public Command resetArmPos(){
        return Commands.runOnce(()->armSubsystem.setState(armSubsystem.shoulderState().getDegrees(), armSubsystem.wristState().getDegrees()));
    }
}
