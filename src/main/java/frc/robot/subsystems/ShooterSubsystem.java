package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.DegreesPerSecond;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkMax shooterTop;
    private final CANSparkMax shooterBottom;

    private final SparkPIDController shooterTopController;
    private final SparkPIDController shooterBottomController;

    private final RelativeEncoder shooterTopEncoder;
    private final RelativeEncoder shooterBottomEncoder;
    private double shooterRPMTolerance;
    private double maxTopSpeed, maxBotSpeed;
    private double desiredTopSpeed, desiredBotSpeed;
	private final Measure<Velocity<Voltage>> rampRate= Volts.of(0.5).per(Seconds.of(1));
	private final Measure<Voltage> stepVoltage = Volts.of(7);
	private final Measure<Time>timeout = Seconds.of(10);
	private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
	private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
	private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
	private final SysIdRoutine ShooterSysIdRoutine;
	private SimpleMotorFeedforward ShooterTopFF,ShooterBotFF;

    public ShooterSubsystem(int shooterTopID, int shooterBottomID, double shooterP,
                            double shooterI, double shooterD, double shooterFF) {
        shooterTop = new CANSparkMax(shooterTopID, CANSparkLowLevel.MotorType.kBrushless);
        shooterBottom = new CANSparkMax(shooterBottomID, CANSparkLowLevel.MotorType.kBrushless);
        shooterTop.setInverted(true);
        shooterBottom.setInverted(true);
        shooterTop.setIdleMode(CANSparkBase.IdleMode.kCoast);
        shooterBottom.setIdleMode(CANSparkBase.IdleMode.kCoast);
//        shooterBottom.setSmartCurrentLimit(38);
//        shooterTop.setSmartCurrentLimit(38);

        shooterTopEncoder = shooterTop.getEncoder();
        shooterTopController = shooterTop.getPIDController();
        shooterTopController.setP(1.9e-10); shooterTopController.setI(0);
        shooterTopController.setD(0); shooterTopController.setFF(0);
        shooterTopController.setOutputRange(0, 1);

        shooterBottomEncoder = shooterBottom.getEncoder();
        shooterBottomController = shooterBottom.getPIDController();
        shooterBottomController.setP(1.9e-10); shooterBottomController.setI(0);
        shooterBottomController.setD(0); shooterBottomController.setFF(0);
        shooterTopController.setOutputRange(0, 1);
        shooterRPMTolerance=5;
        maxBotSpeed = 3500; maxTopSpeed = 3500;
        desiredBotSpeed = 0; desiredTopSpeed = 0;
		shooterTop.setSmartCurrentLimit(40);
	    shooterBottom.setSmartCurrentLimit(40);
		ShooterTopFF=new SimpleMotorFeedforward(0.40322,0.002178,0.001054);
		ShooterBotFF=new SimpleMotorFeedforward(0.22312,0.002178,0.00078615);
		//SYS ID for bottom shooter
	    ShooterSysIdRoutine = new SysIdRoutine(
			    new SysIdRoutine.Config(rampRate,stepVoltage,timeout),
			    new SysIdRoutine.Mechanism(
					    (Measure<Voltage> volts)->{
						    shooterBottom.setVoltage(volts.in(Volts));
					    },
					    log -> {
						    log.motor("shooter-bottom-motor").voltage(
								    m_appliedVoltage.mut_replace(
										    shooterBottom.getAppliedOutput()*shooterBottom.getBusVoltage(), Volts)
						    ).angularPosition(m_angle.mut_replace(
								    shooterBottomEncoder.getPosition(), Rotations)
						    ).angularVelocity(m_velocity.mut_replace(
								    (shooterBottomEncoder.getVelocity()/60),RotationsPerSecond));//Angular Velocity in Rotations Per Second
					    },
					    this
			    )
	    );
    }
    @Override
    public void periodic(){
        SmartDashboard.putNumber("shooterTopSpeed", getShooterSpeedTop());
        SmartDashboard.putNumber("shooterBotSpeed", getShooterSpeedBottom());
		SmartDashboard.putNumber("shooterTopVoltage",shooterTop.getAppliedOutput()*shooterTop.getBusVoltage());
	    SmartDashboard.putNumber("shooterBotVoltage",shooterBottom.getAppliedOutput()*shooterBottom.getBusVoltage());
        SmartDashboard.putBoolean("shooterOn", getMaxShooterSpeedTop() != 0);
    }
    public void setShooterTopSpeed(double speed){
        setDesiredTopSpeed(speed);
        SmartDashboard.putNumber("shooterTopDesired", speed);
        shooterTopController.setReference(speed, CANSparkBase.ControlType.kVelocity,1,ShooterTopFF.calculate(speed));
        //shooterTop.set(speed);
    }
    public void setShooterBottomSpeed(double speed){
        setDesiredBotSpeed(speed);
        SmartDashboard.putNumber("shooterBottomDesired", speed);
        shooterBottomController.setReference(speed, CANSparkBase.ControlType.kVelocity,1,ShooterBotFF.calculate(speed));
        //shooterBottom.set(speed);
    }
    public void setShooterSpeeds(double topSpeed, double bottomSpeed){
        setShooterTopSpeed(topSpeed); setShooterBottomSpeed(bottomSpeed);
    }
    public void setMaxShooterSpeeds(double topSpeed, double bottomSpeed){
        setMaxShooterSpeedTop(topSpeed);
        setMaxShooterSpeedBot(bottomSpeed);
    }
    public void setDesiredTopSpeed(double speed){
        desiredTopSpeed = speed;
    }
    public void setDesiredBotSpeed(double speed){
        desiredBotSpeed = speed;
    }

    public double getDesiredTopSpeed(){
        return desiredTopSpeed;
    }
    public double getDesiredBotSpeed(){
        return desiredBotSpeed;
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

    public double getMaxShooterSpeedTop(){
        return maxTopSpeed;
    }
    public double getMaxShooterSpeedBot(){
        return maxBotSpeed;
    }

    public void setMaxShooterSpeedTop(double speed){
        maxTopSpeed = speed;
    }
    public void setMaxShooterSpeedBot(double speed){
        maxBotSpeed = speed;
    }

    public void setShooterRPMTolerance(double Tolerance){
        shooterRPMTolerance=Tolerance;
        SmartDashboard.putNumber("Shooter RPM Tolerance", Tolerance);
    }

    public double getShooterRPMTolerance(){
        return shooterRPMTolerance;
    }

    public boolean shooterAtSpeed(){
        return ((Math.abs(getDesiredTopSpeed() - getShooterSpeedTop()) <= getShooterRPMTolerance()) &&
                (Math.abs(getDesiredBotSpeed() - getShooterSpeedBottom()) <= getShooterRPMTolerance()));
    }

	public Command shooterQuasiStatic(SysIdRoutine.Direction direction){
		return ShooterSysIdRoutine.quasistatic(direction).handleInterrupt(()->setShooterTopSpeed(0)).andThen(()->setShooterTopSpeed(0));
	}

	public Command shooterDynamic(SysIdRoutine.Direction direction){
		return ShooterSysIdRoutine.dynamic(direction).handleInterrupt(()->setShooterTopSpeed(0)).andThen(()->setShooterTopSpeed(0));
	}
}