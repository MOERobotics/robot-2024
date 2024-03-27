package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public abstract class GenericContainer {

    public final Gyroscope gyroscope;
    public final CollectorSubsystem collectorSubsystem;
    public final ShooterSubsystem shooterSubsystem;
    public final Climber climber;
    public final SwerveDrive swerveSubsystem;

    public GenericContainer(Gyroscope gyro, CollectorSubsystem collectorSubsystem, ShooterSubsystem shooterSubsystem
            , Climber climberSubsystem, SwerveDrive swerveSubsystem, ArmSubsystem armSubsystem){
        this.gyroscope = gyro;
        this.collectorSubsystem = collectorSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.climber = climberSubsystem;
        this.swerveSubsystem = swerveSubsystem;

    }




}
