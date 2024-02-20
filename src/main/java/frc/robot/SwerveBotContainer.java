// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.SwerveController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.setHeading;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class SwerveBotContainer extends RobotContainer {
    // public Solenoid shooter;

    public DigitalOutput shooter;

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
            19,
            18,
            34,
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
            33,
            false,
            true,
            -135,
            new Translation2d(-width, -length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveModule frontLeftModule = new SwerveModule(
            11,
            10,
            31,
            false,
            true,
            45,
            new Translation2d(width, length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveModule frontRightModule = new SwerveModule(
            9,
            8,
            32,
            false,
            true,
            -45,
            new Translation2d(width, -length),
            encoderTicksPerMeter,velocityConversionFactor, pivotP, pivotI, pivotD,
            driveP, driveI, driveD, driveFF
    );
    private final SwerveDrive swerveSubsystem = new SwerveDrive(frontLeftModule, backLeftModule, frontRightModule, backRightModule,
            ()->pigeon.getYaw(), maxMPS,0.04,0,0);
    /////////////////////////////////////////////////////////////////////////////drive subsystems end

    private final Joystick driverJoystick = new Joystick(1); ///joystick imports
    private final Joystick funcOpJoystick = new Joystick(0);

    ////////////////////////////////////////////////////////////////////////////commands

    private final Command drive  = new SwerveController(swerveSubsystem,
            () -> -driverJoystick.getRawAxis(1),
            () -> -driverJoystick.getRawAxis(0),
            () -> -driverJoystick.getRawAxis(2),
            () -> driverJoystick.getRawButton(6),
            () -> driverJoystick.getRawButton(1), 6,6, maxMPS, maxRPS
    );

    ////////////////////////////////////////////////////////////////////////////commands end




    public SwerveBotContainer() {

        shooter = new DigitalOutput(4);

        pigeon.reset();
        swerveSubsystem.setDefaultCommand(drive);
        // Configure the trigger bindings
        configureBindings();
        var button8 = new Trigger(()->driverJoystick.getRawButton(8)); //turn to source
        button8.whileTrue(new setHeading(swerveSubsystem,
                () -> -driverJoystick.getRawAxis(1),
                () -> -driverJoystick.getRawAxis(0),60*((DriverStation.getAlliance().get()==DriverStation.Alliance.Red)?1:-1)));

        var button7 = new Trigger(()->driverJoystick.getRawButton(7)); //turn to amp
        button7.whileTrue(new setHeading(swerveSubsystem,
                () -> -driverJoystick.getRawAxis(1),
                () -> -driverJoystick.getRawAxis(0),90*((DriverStation.getAlliance().get()==DriverStation.Alliance.Red)?-1:1)));
    }

    @Override
    protected void configureBindings() {
        new JoystickButton(driverJoystick, 1).onTrue(Commands.runOnce(() -> {pigeon.setYaw(0); swerveSubsystem.setDesiredYaw(0);}));
        var loop = CommandScheduler.getInstance().getDefaultButtonLoop();
            new Trigger(funcOpJoystick.axisGreaterThan(3, 0.8, loop))
                    .whileTrue(Commands.runOnce(() -> shooter.set(true))).whileFalse(Commands.runOnce(()->shooter.set(false)));
    }

    private void shooterOn(Solenoid shooter) {
        shooter.set(true);
    }

    public Command getAutonomousCommand() {
        return null;
        // return Autos.exampleAuto(m_drive);
    }
}













