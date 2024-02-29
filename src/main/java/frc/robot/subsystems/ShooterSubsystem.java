package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterTop;
    private final CANSparkMax shooterBottom;

    private final SparkPIDController shooterTopController;
    private final SparkPIDController shooterBottomController;

    private final RelativeEncoder shooterTopEncoder;
    private final RelativeEncoder shooterBottomEncoder;
    private double shooterSpeedTop=0;//Store desired speeds
    private double shooterSpeedBottom=0;
    private double shooterRPMTolerance=0;
    public ShooterSubsystem(int shooterTopID, int shooterBottomID, double shooterP,
                            double shooterI, double shooterD, double shooterFF) {
        shooterTop = new CANSparkMax(shooterTopID, CANSparkLowLevel.MotorType.kBrushless);
        shooterBottom = new CANSparkMax(shooterBottomID, CANSparkLowLevel.MotorType.kBrushless);
        shooterTop.setInverted(true);
        shooterBottom.setInverted(true);
        shooterTop.setIdleMode(CANSparkBase.IdleMode.kCoast);
        shooterBottom.setIdleMode(CANSparkBase.IdleMode.kCoast);

        shooterTopEncoder = shooterTop.getEncoder();
        shooterTopController = shooterTop.getPIDController();
        shooterTopController.setP(shooterP); shooterTopController.setI(shooterI);
        shooterTopController.setD(shooterD); shooterTopController.setFF(shooterFF);
        shooterTopController.setOutputRange(0, 1);

        shooterBottomEncoder = shooterBottom.getEncoder();
        shooterBottomController = shooterBottom.getPIDController();
        shooterBottomController.setP(shooterP); shooterBottomController.setI(shooterI);
        shooterBottomController.setD(shooterD); shooterBottomController.setFF(shooterFF);
        shooterTopController.setOutputRange(0, 1);
        shooterRPMTolerance=5;
    }
    public void setShooterTopSpeed(double speed){
        shooterSpeedTop=speed;
        SmartDashboard.putNumber("shooterTopDesired", speed);
        shooterTopController.setReference(speed, CANSparkBase.ControlType.kVelocity);
    }
    public void setShooterBottomSpeed(double speed){
        shooterSpeedBottom=speed;
        SmartDashboard.putNumber("shooterBottomDesired", speed);
        shooterBottomController.setReference(speed, CANSparkBase.ControlType.kVelocity);
    }
    public void setShooterSpeeds(double topSpeed, double bottomSpeed){
        setShooterTopSpeed(topSpeed); setShooterBottomSpeed(bottomSpeed);
    }

    public void stopShooter(){
        setShooterSpeeds(0,0);
    }
    public double getShooterSpeedTop(){
        return shooterTopEncoder.getVelocity();
    }
    public double getShooterSpeedBottom(){
        return shooterBottomEncoder.getVelocity();
    }

    public void setShooterRPMTolerance(double Tolerance){
        shooterRPMTolerance=Tolerance;
        SmartDashboard.putNumber("Shooter RPM Tolerance", Tolerance);
    }

    public double getShooterRPMTolerance(){
        return shooterRPMTolerance;
    }

    public boolean shooterAtSpeed(){
        return ((Math.abs(getShooterSpeedTop() - getShooterSpeedTop()) <= getShooterRPMTolerance()) &&
                (Math.abs(getShooterSpeedBottom() - getShooterSpeedBottom()) <= getShooterRPMTolerance()));
    }
}