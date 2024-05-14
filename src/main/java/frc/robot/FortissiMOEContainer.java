// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.autos.doubleNoteAutos;
import frc.robot.commands.autos.tripleNoteAutos;
import frc.robot.subsystems.*;
import frc.robot.vision.Vision;

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
            0 * ClimberArm.CONVERSION_FACTOR_INCHES,
            0 * ClimberArm.CONVERSION_FACTOR_INCHES,
            3.94 * ClimberArm.CONVERSION_FACTOR_INCHES,
            3.57 * ClimberArm.CONVERSION_FACTOR_INCHES,
            0.52 * ClimberArm.CONVERSION_FACTOR_INCHES,
            0.83 * ClimberArm.CONVERSION_FACTOR_INCHES
    );
    public AHRS navx = new AHRS(I2C.Port.kMXP, (byte)50);

    WPI_Pigeon2 pigeon = new WPI_Pigeon2(0);
    PowerDistribution pdh = new PowerDistribution(21, PowerDistribution.ModuleType.kRev);

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
    double maxMPSAuto = 4;
    double maxRPS =  1.5*2*Math.PI;
    double maxRPS2 = Math.PI;

    double maxMPSSquared = 3;

    Vision vision = new Vision();
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

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
            pigeon, maxMPSAuto, maxMPS, maxMPSSquared, maxRPS, maxRPS2,1.0, 0, 0, 1.0, 0, 0, 8e-2, 0,1e-2);
    /////////////////////////////////////////////////////////////////////////////drive subsystems end

    /////////////////////////////////////////////////////////////////////////////arm subsystem start
    private final Arm armSubsystem = new Arm(4, 15,14, 35, 37,
            24.0e-2, 24.0e-3, 48.0e-4, .1724,0/*3.0e-2*/, 2.0e-3,
            .0185, .14833, 1.42e-4, 1.36e-4,
            .95908/3, .54837/3, .033244/3, .00498/3,
            .6048, .3615, .18133,.14154,10.725,27.837,
            5.6705,5.899,0,0,
            Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(-60), Rotation2d.fromDegrees(123), Rotation2d.fromDegrees(-60),
            100,300);

    /////////////////////////////////////////////////////////////////////////// arm subsystem end

    ///////////////////////////////////////////////////////////////////////////////////////head subsystem
	private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(5,
            13,1.9e-10, 0,0,0.40322,0.002178,
            0.001054, 0.22312,0.002178,0.00078615);
    private final CollectorSubsystem collectorSubsystem = new CollectorSubsystem(6,
            0.01,0,0,0,7);
    ///////////////////////////////////////////////////////////////////////////////////////head subsystem


    private final XboxController driverJoystick = new XboxController(1); ///joystick imports TESTING OUT RUMBLE CHANGE TO PS5 CONTROLLER IF ANYTHING BREAKS
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


    Command collectorCommand = new CollectorControllerCommand(
            0.35,//.45
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
            1,
            ()-> -pigeon.getPitch()
    );


    private final Command climbDown= new ClimbDown(
            climber,
            1
    );


    boolean climbingUp;

    boolean climbingDown;


    public final Command buttonsCommand = Commands.run(() -> {
        if (driverJoystick.getRawButtonPressed(6)) {
            CommandScheduler.getInstance().cancel(climbDown);
            climbUp.schedule();
        }
        if (driverJoystick.getRawButtonPressed(5)) {
            CommandScheduler.getInstance().cancel(climbUp);
            climbDown.schedule();
        }
        if (!driverJoystick.getRawButton(5) && !driverJoystick.getRawButton(6)){
            CommandScheduler.getInstance().cancel(climbDown);
            CommandScheduler.getInstance().cancel(climbUp);
            climber.stopArms();
        }
        SmartDashboard.putBoolean("ClimbingDown", climbingDown);
        SmartDashboard.putBoolean("ClimbingUp", climbingUp);

        SmartDashboard.putNumber("Roll", pigeon.getRoll());
        SmartDashboard.putNumber("Pitch", pigeon.getPitch());
        pdh.setSwitchableChannel(((collectorSubsystem.isCollected() || collectorSubsystem.getCollectorAmps() > 20) && ((System.currentTimeMillis()/100)%2 == 0))
                ||  (shooterSubsystem.shooterAtSpeed() && shooterSubsystem.getDesiredTopSpeed() != 0 && (collectorSubsystem.isCollected() || collectorSubsystem.getCollectorAmps() > 20)));
    });
    //weirdest command ever - climbing & pdh logic

    ////////////////////////////////////////////////////////////////////////////commands end

    Command shooterControl = new ShooterControllerCommand(shooterSubsystem, armSubsystem::getShoulderDesState,
            ()->functionJoystick.getRawButtonPressed(5));
    ////////////////////////////////////////////////////////////////////////////commands end
    Trigger shoulderUp = new Trigger(()-> functionJoystick.getRawAxis(1) <= -.3);
    Trigger shoulderDown = new Trigger(()-> functionJoystick.getRawAxis(1) >= .3);

    Trigger wristUp = new Trigger(()-> functionJoystick.getRawAxis(0) >= .3);
    Trigger wristDown = new Trigger(()-> functionJoystick.getRawAxis(0) <= -.3);


    public FortissiMOEContainer() {
        shooterSubsystem.setShooterRPMTolerance(250);
        shooterSubsystem.setMaxShooterSpeeds(3200,3200);

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

        m_chooser.setDefaultOption("Double Note Auto 1 (CW2)", new doubleNoteAutos(swerveSubsystem,armSubsystem,shooterSubsystem, collectorSubsystem,0,0).DoubleNoteAuto1());
        m_chooser.addOption("Double Note Auto 2 (BW1)", new doubleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0,0).DoubleNoteAuto2());
       // m_chooser.addOption("Double Note Auto 3 (CW1)", new doubleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0,0).DoubleNoteAuto3());
        m_chooser.addOption("Double Note Auto 4 (DW3)", new doubleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0,0).DoubleNoteAuto4());
       // m_chooser.addOption("Triple Note Auto (BW1W2)", new tripleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0,0).BW1W2());
        m_chooser.addOption("A Move Auto", new doubleNoteAutos(swerveSubsystem,armSubsystem,shooterSubsystem,collectorSubsystem,0,0).AMoveAuto());
        m_chooser.addOption("Triple Note CW1W2", new tripleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0,0).CW1W2());
        m_chooser.addOption("Double Note Auto 1 Return Sub (CW2)", new doubleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0,0).DoubleNoteAuto1ScoreSub());
        m_chooser.addOption("Double Note Auto 2 Return Sub (BW1)", new doubleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0,0).DoubleNoteAuto2ScoreSub());
        m_chooser.addOption("Double Note Auto 4 Return Sub (DW3)", new doubleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0,0).DoubleNoteAuto4ScoreSub());
        m_chooser.addOption("D Score Move", new doubleNoteAutos(swerveSubsystem,armSubsystem,shooterSubsystem,collectorSubsystem,0,0).DMoveAuto());
        m_chooser.addOption("D Score Collect (DC5)", new doubleNoteAutos(swerveSubsystem,armSubsystem,shooterSubsystem,collectorSubsystem,0,0).DC5Auto());
        m_chooser.addOption("4 Note Auto (CW2W1W3)", new tripleNoteAutos(swerveSubsystem,armSubsystem,shooterSubsystem,collectorSubsystem,0,0).CW1W2W3());
        m_chooser.addOption("CenterLine Pass Auto (DC5C4PassC3)", new doubleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0, 0).DC5C4PassC3());
        m_chooser.addOption("driveForward", new doubleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0, 0).rollOutAuto());
        m_chooser.addOption("3 Note Centerline Auto (DC3C2)", new tripleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0, 0).DC3C2());
        m_chooser.addOption("2 Note Centerline Auto Obj Detect", new tripleNoteAutos(swerveSubsystem, armSubsystem, shooterSubsystem, collectorSubsystem, 0,0).DC3ObjDetect());
        SmartDashboard.putData("chooser", m_chooser);
    }


    private void configureBindings() {


        new JoystickButton(driverJoystick, 1).onTrue(Commands.runOnce(() -> {pigeon.setYaw(0); swerveSubsystem.setDesiredYaw(0);}));
        // zero heading

        shoulderUp.whileTrue(Commands.run(()->armSubsystem.shoulderVoltageController(1.5)));
        wristUp.whileTrue(Commands.run(()->armSubsystem.wristVoltageController(1.5))); //in volts lol
        shoulderDown.whileTrue(Commands.run(()->armSubsystem.shoulderVoltageController(-1.5)));
        wristDown.whileTrue(Commands.run(()->armSubsystem.wristVoltageController(-1.5)));
        //manual shoulder/wrist control

        new JoystickButton(functionJoystick, 1).onTrue(Commands.defer(()->armSubsystem.goToPoint(Constants.collectorShoulder, Constants.collectorWrist), Set.of(armSubsystem))
                .until(()->(shoulderUp.getAsBoolean() || functionJoystick.getRawButton(3) ||
                        functionJoystick.getRawButton(8) || shoulderDown.getAsBoolean() ||
                        functionJoystick.getRawButton(2)|| wristDown.getAsBoolean()|| wristUp.getAsBoolean()
                        || functionJoystick.getRawButton(10) || buttonBox.getRawButton(3))));
        //collect

        new JoystickButton(functionJoystick, 8).onTrue(Commands.defer(() ->armSubsystem.goToPoint(Rotation2d.fromDegrees(120), Rotation2d.fromDegrees(-44)), Set.of(armSubsystem))
                .until(()->(shoulderUp.getAsBoolean() || functionJoystick.getRawButton(3) ||
                        functionJoystick.getRawButton(2) || shoulderDown.getAsBoolean() ||
                        functionJoystick.getRawButton(1)||wristDown.getAsBoolean()|| wristUp.getAsBoolean()
                        || functionJoystick.getRawButton(10) || buttonBox.getRawButton(3))));
        //all the shots smh until roshik chooses a new random button to be his favorite

        new JoystickButton(functionJoystick, 10).onTrue(Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(147), Rotation2d.fromDegrees(-78)), Set.of(armSubsystem))
                .until(()->(shoulderUp.getAsBoolean() || functionJoystick.getRawButton(3) ||
                        functionJoystick.getRawButton(2) || shoulderDown.getAsBoolean() ||
                        functionJoystick.getRawButton(1) || functionJoystick.getRawButton(8) || wristDown.getAsBoolean()
                        || wristUp.getAsBoolean() || buttonBox.getRawButton(3))));
        //amp shot

        new JoystickButton(functionJoystick, 3).onTrue(Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(113), Rotation2d.fromDegrees(-110)), Set.of(armSubsystem))
                .until(()->(shoulderUp.getAsBoolean() || functionJoystick.getRawButton(10) ||
                        functionJoystick.getRawButton(2) || shoulderDown.getAsBoolean() ||
                        functionJoystick.getRawButton(1) || functionJoystick.getRawButton(8)|| wristDown.getAsBoolean()
                        || wristUp.getAsBoolean() || buttonBox.getRawButton(3))));
        //safe pos

        new JoystickButton(buttonBox, 3).onTrue(Commands.defer(()->armSubsystem.goToPoint(Rotation2d.fromDegrees(135), Rotation2d.fromDegrees(-135)), Set.of(armSubsystem))
                .until(()->(shoulderUp.getAsBoolean() || functionJoystick.getRawButton(3) ||
                        functionJoystick.getRawButton(2) || shoulderDown.getAsBoolean() ||
                        functionJoystick.getRawButton(1) || functionJoystick.getRawButton(8)|| wristDown.getAsBoolean()
                        || wristUp.getAsBoolean() || functionJoystick.getRawButton(10))));
        //start position


        new JoystickButton(functionJoystick, 2).onTrue(
                Commands.parallel(
                Commands.defer(()->armSubsystem.goToPoint(
                Rotation2d.fromDegrees(armSubsystem.autoAim(swerveSubsystem::getEstimatedPose).getX()),
                Rotation2d.fromDegrees(armSubsystem.autoAim(swerveSubsystem::getEstimatedPose).getY())), Set.of(armSubsystem)).andThen(
                        Commands.run(()-> armSubsystem.holdPos(armSubsystem.getShoulderDesState(), armSubsystem.getWristDesState())
                        ))
                        .until(()->(shoulderUp.getAsBoolean() || functionJoystick.getRawButton(10) ||
                                functionJoystick.getRawButton(3) || shoulderDown.getAsBoolean() ||
                                functionJoystick.getRawButton(1) || functionJoystick.getRawButton(8)|| wristUp.getAsBoolean()
                                || wristDown.getAsBoolean() || buttonBox.getRawButton(3))),
                        Commands.run(()->swerveSubsystem.setDesiredYaw(swerveSubsystem.getAngleBetweenSpeaker(
                                ()->swerveSubsystem.getEstimatedPose().getTranslation()).getDegrees())).until(
                                ()->Math.abs(driverJoystick.getRawAxis(2)) >= .1
                        )));
        //auto aim shot

        new JoystickButton(functionJoystick, 4).whileTrue(Commands.runOnce(()->shooterSubsystem.setMaxShooterSpeeds(2300,2300))).
                whileFalse(Commands.runOnce(()->shooterSubsystem.setMaxShooterSpeeds(3200,3200)));
        //half speed reduction

        var driveToNote = new DriveToNoteCommand(
                swerveSubsystem,
                vision,
//                () -> Math.max(0, Math.hypot(driverJoystick.getRawAxis(0), driverJoystick.getRawAxis(1))-.05)*(maxMPS),
		        () -> -driverJoystick.getRawAxis(1),
		        () -> -driverJoystick.getRawAxis(0),
		        () -> -driverJoystick.getRawAxis(2),
                (rumblePercent) -> {
                    SmartDashboard.putNumber("JoyRumble", rumblePercent);
                    driverJoystick.setRumble(PS5Controller.RumbleType.kBothRumble, rumblePercent); //TODO: try different rumble types.
                },
		        maxMPS
        );
        new JoystickButton(driverJoystick, 8).whileTrue(driveToNote);
        // object detection note pickup button

    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
        // return Autos.exampleAuto(m_drive);
    }
    public Command resetArmPos(){
        return Commands.runOnce(()->armSubsystem.setState(armSubsystem.shoulderState().getDegrees(), armSubsystem.wristState().getDegrees()));
    }
}
