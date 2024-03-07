// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.commands.*;
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

    public Climber climber = new Climber(
            7,
            12,
            1,
            0,
            false,
            true,
            0.52 * ClimberArm.CONVERSION_FACTOR_INCHES,
            0.42 * ClimberArm.CONVERSION_FACTOR_INCHES,
            3.76 * ClimberArm.CONVERSION_FACTOR_INCHES,
            3.57 * ClimberArm.CONVERSION_FACTOR_INCHES,
            0.52 * ClimberArm.CONVERSION_FACTOR_INCHES,
            0.65 * ClimberArm.CONVERSION_FACTOR_INCHES
    );
    public AHRS navx = new AHRS(I2C.Port.kMXP, (byte)50);

    WPI_Pigeon2 pigeon = new WPI_Pigeon2(0);

    /////////////////////////////////////////////////////////////////////////////drive subsystems
    double encoderTicksPerMeter = 6.75/12.375*1.03/1.022*39.3701;
    double velocityConversionFactor = 32.73*1.03/1.022 * Units.metersToInches(1);
    double pivotP = 8.0e-3*60;
    double pivotI = 0.0;
    double pivotD = 0.0;
    double driveP = 5.0e-5;
    double driveI = 0.0;
    double driveD = 2.0e-4;
    double driveFF = 1.76182e-4;
    double width = Units.inchesToMeters(14);
    double length = Units.inchesToMeters(14);
    double maxMPS = 174/39.3701;
    double maxRPS =  Math.PI;
    double maxRPS2 = Math.PI;

    double maxMPSSquared = 2.5;
    private final SwerveModule backLeftModule = new SwerveModule(
            19,
            18,
            31,
            false,
            true,
            135,
            new Translation2d(-width, length),
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
            new Translation2d(-width, -length),
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
            new Translation2d(width, length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveModule frontRightModule = new SwerveModule(
            3,
            2,
            33,
            false,
            true,
            -45,
            new Translation2d(width, -length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveDrive swerveSubsystem = new SwerveDrive(frontLeftModule, backLeftModule, frontRightModule, backRightModule,
            pigeon, maxMPS, maxMPSSquared, maxRPS, maxRPS2,1.0, 0, 0, 1.0, 0, 0, 4e-2, 0,0);
    /////////////////////////////////////////////////////////////////////////////drive subsystems end
    /////////////////////////////////////////////////////////////////////////////arm subsystem start
    private final Arm armSubsystem = new Arm(4, 15,14, 35, 36,
            1.0e-2, 1.0e-3, 1.0e-4, 5.0e-2, 5.0e-3, 5.0e-4, 1.0e-2,1.0e-3,0,0, 0, Rotation2d.fromDegrees(102),
            Rotation2d.fromDegrees(-53), 100,30);

    /////////////////////////////////////////////////////////////////////////// arm subsystem end

    ///////////////////////////////////////////////////////////////////////////////////////head subsystem
	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(5,
            13,1.0e-4, 0,0,driveFF);
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
            () -> -driverJoystick.getRawAxis(2),
            () -> driverJoystick.getRawButton(5),
            () -> driverJoystick.getRawButton(3), 2,3, maxMPS, maxRPS
    );

    Command turnToAmp = new setHeading( swerveSubsystem,
            ()-> -driverJoystick.getRawAxis(1),
            ()-> -driverJoystick.getRawAxis(0),
            ()->AllianceFlip.apply(Rotation2d.fromDegrees(90)));
    Command turnToSource = new setHeading(swerveSubsystem,
            ()-> -driverJoystick.getRawAxis(1),
            ()-> -driverJoystick.getRawAxis(0),
            ()->AllianceFlip.apply(Rotation2d.fromDegrees(-60)));

    // private final Command turnRobotOn = new CollectorOnOrOffCommand(headSubsystem, true);
    Command collectorCommand = new CollectorControllerCommand(
            0.45,
            ()->functionJoystick.getRawAxis(2)>=0.5,
            ()->functionJoystick.getRawAxis(3)>=0.5,
            ()->functionJoystick.getRawButton(6),
            collectorSubsystem
    );



    private final Command moveArms= new TestClimber(
            climber,
            () -> buttonBox.getRawButton(7),
            () -> buttonBox.getRawButton(8),
            () -> buttonBox.getRawButton(4),
            () -> buttonBox.getRawButton(5)

    );


    private final Command climbUp= new ClimbUp(
            climber,
            0.7,
            ()-> -pigeon.getPitch()
    );


    private final Command climbDown= new ClimbDown(
            climber,
            0.8
    );


    boolean climbingUp;

    boolean climbingDown;


    public final Command buttonsCommand = Commands.run(() -> {
        SmartDashboard.putData("lmao", climbUp);
        if (driverJoystick.getRawButtonPressed(6)) {
            if (climbingUp) {
                CommandScheduler.getInstance().cancel(climbUp);
                this.climbingUp = false;
            } else {
                climbUp.schedule();
                this.climbingUp = true;
                SmartDashboard.putBoolean("ClimbingUp", climbingUp);
            }
        }
        if (driverJoystick.getRawButtonPressed(5)) {
            if (climbingDown) {
                CommandScheduler.getInstance().cancel(climbDown);
                this.climbingDown = false;
                SmartDashboard.putBoolean("ClimbingDown", climbingDown);
            } else {
                climbDown.schedule();
                this.climbingDown = true;
            }
        }
        SmartDashboard.putBoolean("ClimbingDown", climbingDown);
        SmartDashboard.putBoolean("ClimbingUp", climbingUp);

        SmartDashboard.putNumber("Roll", pigeon.getRoll());
        SmartDashboard.putNumber("Pitch", pigeon.getPitch());

    });

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

        climber.setDefaultCommand(moveArms);

        //collectorSubsystem.setDefaultCommand(collectorCommand);
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
        SmartDashboard.putString("hi","hi");

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
                        functionJoystick.getRawButton(2)||buttonBox.getRawButton(1)|| buttonBox.getRawButton(2))));
        //collect

        new JoystickButton(functionJoystick, 2).onTrue(Commands.defer(() -> armSubsystem.goToPoint(Rotation2d.fromDegrees(113.5), Rotation2d.fromDegrees(-51.19)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(4) || functionJoystick.getRawButton(1) ||
                        functionJoystick.getRawButton(8)||buttonBox.getRawButton(1)|| buttonBox.getRawButton(2))));
        //podium shot

        new JoystickButton(functionJoystick, 4).onTrue(Commands.defer(() ->armSubsystem.goToPoint(Rotation2d.fromDegrees(109), Rotation2d.fromDegrees(-35)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(2) || functionJoystick.getRawButton(8) ||
                        functionJoystick.getRawButton(1)||buttonBox.getRawButton(1)|| buttonBox.getRawButton(2))));
        //wing shot

        new JoystickButton(functionJoystick, 3).onTrue(Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(114), Rotation2d.fromDegrees(-102.5)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(2) || functionJoystick.getRawButton(8) ||
                        functionJoystick.getRawButton(1) || functionJoystick.getRawButton(4)||buttonBox.getRawButton(1)|| buttonBox.getRawButton(2))));
        //mid shot

        new JoystickButton(buttonBox, 3).onTrue(Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-135)), Set.of(armSubsystem))
                .until(()->(functionJoystick.getRawButton(7) || functionJoystick.getRawButtonPressed(3) ||
                        functionJoystick.getRawButtonPressed(2) || functionJoystick.getRawButton(8) ||
                        functionJoystick.getRawButton(1) || functionJoystick.getRawButton(4)||buttonBox.getRawButton(1)|| buttonBox.getRawButton(2))));
        //start position

//        new JoystickButton(driverJoystick, 7).onTrue(turnToAmp.until(()->(Math.abs(driverJoystick.getRawAxis(2)) >= .2)));
//        new JoystickButton(driverJoystick, 8).onTrue(turnToSource.until(()->(Math.abs(driverJoystick.getRawAxis(2)) >= .2)));

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
        return new doubleNoteAutos(swerveSubsystem,armSubsystem,shooterSubsystem,collectorSubsystem,0,0).DoubleNoteAuto3();
        // return Autos.exampleAuto(m_drive);
    }
    public Command resetArmPos(){
        return Commands.runOnce(()->armSubsystem.setState(armSubsystem.shoulderState().getDegrees(), armSubsystem.wristState().getDegrees()));
    }
}
