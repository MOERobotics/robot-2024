// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.*;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HeadSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private final CANSparkMax shooterTop;
    private final CANSparkMax shooterBottom;
    private final CANSparkMax collector;
    private final RelativeEncoder shooterTopEncoder;
    private final RelativeEncoder shooterBottomEncoder;
    private final SparkPIDController shooterTopController;
    private final SparkPIDController shooterBottomController;
    private final SparkPIDController collectorController;
    private final DigitalInput collectorBeam;
    private final DoubleLogEntry logCollectorSpeed;
    private final DoubleLogEntry logCollectorPower;
    private final DoubleLogEntry logShooterTopSpeed;
    private final DoubleLogEntry logShooterBottomSpeed;
    private final DoubleLogEntry logShooterTopDesired;
    private final DoubleLogEntry logShooterBottomDesired;
    private final DoubleLogEntry logShooterTolerance;

    private double shooterSpeedTop=0;//Store desired speeds
    private double shooterSpeedBottom=0;
    private boolean collectorState;
    private int shooterRPMTolerance;
    public HeadSubsystem(int shooterTopID, int shooterBottomID, int collectorID, double shooterP, double shooterI, double shooterD, double shooterFF,
                         double collectorP, double collectorI, double collectorD, double collectorFF,  int collectorBeamID) {
        //instantiate shooter motors, encoders, sensors, PID
        this.collectorBeam = new DigitalInput(collectorBeamID);
        this.shooterTop=new CANSparkMax(shooterTopID, CANSparkLowLevel.MotorType.kBrushless);
        this.shooterBottom=new CANSparkMax(shooterBottomID,CANSparkLowLevel.MotorType.kBrushless);
        this.collector=new CANSparkMax(collectorID, CANSparkLowLevel.MotorType.kBrushless);
        shooterTop.setIdleMode(CANSparkBase.IdleMode.kCoast);
        shooterBottom.setIdleMode(CANSparkBase.IdleMode.kCoast);
        collector.setIdleMode(CANSparkBase.IdleMode.kBrake);
        //TODO: Reverse motors if needed
        shooterTop.setInverted(false);
        shooterBottom.setInverted(false);
        collector.setInverted(false);

        this.shooterTopEncoder = shooterTop.getEncoder();
        this.shooterBottomEncoder = shooterBottom.getEncoder();

        var log = DataLogManager.getLog();
        logCollectorSpeed = new DoubleLogEntry(log, "Head/collectorSpeed");
        logCollectorPower = new DoubleLogEntry(log, "Head/colelctorPower");
        logShooterTopSpeed = new DoubleLogEntry(log, "Head/TopShooterSpeed");
        logShooterBottomSpeed = new DoubleLogEntry(log, "Head/BottomShooterSpeed");
        logShooterTopDesired = new DoubleLogEntry(log, "Head/TopShooterDesired");
        logShooterBottomDesired = new DoubleLogEntry(log, "Head/BottomShooterDesired");
        logShooterTolerance = new DoubleLogEntry(log, "Head/ShooterRPMTolerance");

        // configure collector motor top and bottom

        // set top shooter PID
        this.shooterTopController = shooterTop.getPIDController();
        shooterTopController.setP(shooterP);
        shooterTopController.setI(shooterI);
        shooterTopController.setIZone(0);
        shooterTopController.setD(shooterD);
        shooterTopController.setFF(shooterFF);
        shooterTopController.setOutputRange(-1, 1);

        // set bottom shooter PID
        this.shooterBottomController = shooterBottom.getPIDController();
        shooterBottomController.setP(shooterP);
        shooterBottomController.setI(shooterI);
        shooterBottomController.setIZone(0);
        shooterBottomController.setD(shooterD);
        shooterBottomController.setFF(shooterFF);
        shooterBottomController.setOutputRange(-1, 1);

        // set  Collecter PID
        this.collectorController = collector.getPIDController();
        collectorController.setP(collectorP);
        collectorController.setI(collectorI);
        collectorController.setIZone(0);
        collectorController.setD(collectorD);
        collectorController.setFF(collectorFF);
        collectorController.setOutputRange(-1, 1);

		// sets up tolerance
        setShooterRPMTolerance(5);
    }

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public Command exampleMethodCommand() {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(
				() -> {
					/* one-time action goes here */
				});
	}

	/**
	 * An example method querying a boolean state of the subsystem (for example, a digital sensor).
	 *
	 * @return value of some boolean subsystem state, such as a digital sensor.
	 */
	public boolean exampleCondition() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}

    //Has a note
    public boolean isCollected(){
        return collectorBeam.get();
    }

    public void setShooterTopSpeed(double speed){
        shooterSpeedTop=speed;
        shooterTopController.setReference(speed,CANSparkBase.ControlType.kVelocity);
    }
    public void setShooterBottomSpeed(double speed){
        shooterSpeedBottom=speed;
        shooterBottomController.setReference(speed,CANSparkBase.ControlType.kVelocity);
    }

	public void setShooterPower(double power){
		shooterTop.set(power);
		shooterBottom.set(power);
	}
    // TODO check collector speeds before turning on(check)
    public void setCollectorSpeed(double speed){
        collector.set(speed);

        if(speed==0){
            collectorState = false;
        } else {
            collectorState = true;
        }


    }
    public Command runCollectorCommands (double speed){
        return  Commands.runOnce(() -> this.setCollectorSpeed(speed));
    }

    public double getShooterSpeedTop(){
        return shooterTopEncoder.getVelocity();
    }
    public double getShooterSpeedBottom(){
        return shooterBottomEncoder.getVelocity();
    }

    //Within reasonable range to shoot?
    public boolean inRange() {
        // Vision to April Tag/Odometry.
        return false;
    }

    //Aimed at the speaker
    public boolean aimed() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    //Is in process of aiming?
    public boolean aiming() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

	public boolean shooterAtSpeed(){
		return ((Math.abs(shooterTopEncoder.getVelocity() - shooterSpeedTop) <= shooterRPMTolerance) && (Math.abs(shooterBottomEncoder.getVelocity() - shooterSpeedBottom) <= shooterRPMTolerance));
	}


    //Good to shoot
    public boolean readyShoot() {
        return (shooterAtSpeed() && aimed() && isCollected());
        //if motors up to speed
        //if aimed
        //if see speaker
	    //Has a note
    }



    public void setShooterRPMTolerance(int shooterRPMtolerance){
        this.shooterRPMTolerance = shooterRPMtolerance;
    }

    public int getShooterRPMTolerance(){
        return shooterRPMTolerance;
    }


    //Stop motors
    public void stopShooter(){
        setShooterTopSpeed(0);
        setShooterBottomSpeed(0);
    }


    public boolean readyDeposit(){
        return false;
    }

    public void stopCollector(){
        collector.set(0);
        collectorState = false;
    }


    public boolean getCollectorState (){
        return collectorState;
    }



    // TODO A state where we know its safe to move even if the note isn't completely in the head(not done)
    public void readyToMoveShooter(){
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterTopDesired:",shooterSpeedTop);
        SmartDashboard.putNumber("shooterBottomDesired:",shooterSpeedBottom);
        SmartDashboard.putNumber("shooterTopActualSpeed:",getShooterSpeedTop());
        SmartDashboard.putNumber("shooterBottomActualSpeed:",getShooterSpeedBottom());
	    SmartDashboard.putNumber("Shooter RPM Tolerance", getShooterRPMTolerance());
		SmartDashboard.putBoolean("Note Collected:",isCollected());
		SmartDashboard.putBoolean("In Range:",inRange());
		SmartDashboard.putBoolean("Aimed at speaker:",aimed());
	    SmartDashboard.putBoolean("Aiming in progress:",aiming());
	    SmartDashboard.putBoolean("Ready to shoot:", readyShoot());
        // This method will be called once per scheduler run
        logCollectorSpeed.append(collector.getEncoder().getVelocity());
        logCollectorPower.append(collector.get());
        logShooterTopSpeed.append(getShooterSpeedTop());
        logShooterBottomSpeed.append(getShooterSpeedBottom());
        logShooterTopDesired.append(shooterSpeedTop);
        logShooterBottomDesired.append(shooterSpeedBottom);
        logShooterTolerance.append(getShooterRPMTolerance());

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}