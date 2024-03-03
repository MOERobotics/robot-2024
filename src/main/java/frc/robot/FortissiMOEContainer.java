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
import frc.robot.commands.autos.tripleNoteAutos;
import frc.robot.commands.setHeading;
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
    double maxRPS2 = Math.PI;

    double maxMPSSquared = 2;
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
            pigeon, maxMPS, maxMPSSquared, maxRPS, maxRPS2,1.0, 0, 0, 1.0, 0, 0, .04, 0,0);
    /////////////////////////////////////////////////////////////////////////////drive subsystems end
    /////////////////////////////////////////////////////////////////////////////arm subsystem start
    private final Arm armSubsystem = new Arm(4, 15,14, 35, 36,
            2.0e-2, 2.0e-3, 2.0e-4, 6.0e-2, 6.0e-3, 6.0e-4, 1.0e-2,1.0e-3,0,0, 0, Rotation2d.fromDegrees(103),
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
    private final Joystick buttonBox = new Joystick(2);


    ////////////////////////////////////////////////////////////////////////////commands



    private final Command drive  = new SwerveController(swerveSubsystem,
            () -> -driverJoystick.getRawAxis(1),
            () -> -driverJoystick.getRawAxis(0),
            () -> driverJoystick.getRawAxis(2),
            () -> driverJoystick.getRawButton(5),
            () -> driverJoystick.getRawButton(3), 2,1, maxMPS, maxRPS
    );

    // private final Command turnRobotOn = new CollectorOnOrOffCommand(headSubsystem, true);
    Command collectorCommand = new CollectorControllerCommand(
            0.5,
            ()->functionJoystick.getRawAxis(2)>=0.5,
            ()->functionJoystick.getRawAxis(3)>=0.5,
            ()->functionJoystick.getRawButton(6),
            collectorSubsystem
    );
    ////////////////////////////////////////////////////////////////////////////commands end

    Command shooterControl = new ShooterControllerCommand(shooterSubsystem, armSubsystem::getShoulderDesState,
            ()->functionJoystick.getRawButtonPressed(5));
    Command setHeading = new setHeading(swerveSubsystem, () -> -driverJoystick.getRawAxis(1),
            () -> -driverJoystick.getRawAxis(0), ()->(swerveSubsystem.getAngleBetweenSpeaker(
                    ()->swerveSubsystem.getEstimatedPose().getTranslation())));
    ////////////////////////////////////////////////////////////////////////////commands end



    public FortissiMOEContainer() {
        shooterSubsystem.setShooterRPMTolerance(500);
        swerveSubsystem.setDefaultCommand(drive);
//        collectorSubsystem.setDefaultCommand(collectorCommand);
        armSubsystem.setDefaultCommand(Commands.run(()->armSubsystem.holdPos(armSubsystem.getShoulderDesState(),
                armSubsystem.getWristDesState()), armSubsystem));
//        armSubsystem.setDefaultCommand(Commands.run(()->armSubsystem.stopMotors(), armSubsystem));
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
        new JoystickButton(buttonBox, 1).whileTrue(Commands.run(()->armSubsystem.wristPowerController(.1)));
        new JoystickButton(functionJoystick, 7).whileTrue(Commands.run(()->armSubsystem.shoulderPowerController(-.1)));
        new JoystickButton(buttonBox, 2).whileTrue(Commands.run(()->armSubsystem.wristPowerController(-.1)));
        new JoystickButton(functionJoystick, 1).onTrue(Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(85), Rotation2d.fromDegrees(-41)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(4) || functionJoystick.getRawButton(8) ||
                        functionJoystick.getRawButton(2)||buttonBox.getRawButton(1)|| buttonBox.getRawButton(2))));//collect
        new JoystickButton(functionJoystick, 2).onTrue(Commands.defer(() -> armSubsystem.goToPoint(Rotation2d.fromDegrees(113.5), Rotation2d.fromDegrees(-51.19)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(4) || functionJoystick.getRawButton(1) ||
                        functionJoystick.getRawButton(8)||buttonBox.getRawButton(1)|| buttonBox.getRawButton(2)))); //podium shot
        new JoystickButton(functionJoystick, 4).onTrue(Commands.defer(() ->armSubsystem.goToPoint(Rotation2d.fromDegrees(109), Rotation2d.fromDegrees(-35)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(2) || functionJoystick.getRawButton(8) ||
                        functionJoystick.getRawButton(1)||buttonBox.getRawButton(1)|| buttonBox.getRawButton(2)))); //wing shot
        /*new JoystickButton(functionJoystick, 3).onTrue(Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(117.3), Rotation2d.fromDegrees(-45.86)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(2) || functionJoystick.getRawButton(8) ||
                        functionJoystick.getRawButton(1) || functionJoystick.getRawButton(4)||buttonBox.getRawButton(1)|| buttonBox.getRawButton(2)))); //mid shot*/
        new JoystickButton(functionJoystick, 3).onTrue(Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-135)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(2) || functionJoystick.getRawButton(8) ||
                        functionJoystick.getRawButton(1) || functionJoystick.getRawButton(4)||buttonBox.getRawButton(1)|| buttonBox.getRawButton(2))));//start position

//        new JoystickButton(functionJoystick, 3).onTrue(
//                Commands.parallel(
//                Commands.defer(()->armSubsystem.goToPoint(
//                Rotation2d.fromDegrees(armSubsystem.autoAim(swerveSubsystem::getEstimatedPose).getX()),
//                Rotation2d.fromDegrees(armSubsystem.autoAim(swerveSubsystem::getEstimatedPose).getY())), Set.of(armSubsystem))
//                        .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
//                        functionJoystick.getRawButtonPressed(2) || functionJoystick.getRawButton(8) ||
//                        functionJoystick.getRawButton(1) || functionJoystick.getRawButton(4))),
//                        setHeading.until(()->Math.abs(driverJoystick.getRawAxis(2))>.1))); //auto aim shot
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
