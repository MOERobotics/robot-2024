// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.AllianceFlip;
import frc.robot.UsefulPoints;
import frc.robot.commands.ArmPathFollow;

import java.util.function.Supplier;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

public class Arm extends SubsystemBase {
    private final CANSparkMax shoulderMotorLeft, shoulderMotorRight;
    private final CANSparkMax wristMotor;

    private final CANCoder shoulderEncoder;
    private final CANCoder wristEncoder;

    private final RelativeEncoder shoulderRelEncoder;
    private final RelativeEncoder wristRelEncoder;

    private final PIDController shoulderController;
    private final PIDController wristController;


    private Rotation2d interShoulder, interWrist, highTransitionShoulderAngle, highTransitionWristAngle;
    private double currShoulder, currWrist;
    private double maxSpeed, maxAccel, shoulderLength, wristLength, shoulderCOMLen, wristCOMLen,
    shoulderCOMOffset, wristCOMOffset, shoulderMass, wristMass, shoulderGearing, wristGearing;
    private double wristOffset = 0;
    private double shoulderOffset = 90;
	private final Measure<Velocity<Voltage>>rampRate= Volts.of(0.25).per(Seconds.of(1));
	private final Measure<Voltage> stepVoltage = Volts.of(2);
	private final Measure<Time>timeout = Seconds.of(5);
	private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
	private final MutableMeasure<Angle> m_angle = mutable(Degrees.of(0));
	private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(DegreesPerSecond.of(0));
	private final SysIdRoutine shoulderSysIdRoutine;
    private final ArmFeedforward wristFF, shoulderFF;

    public Arm(int rightShoulderMotorID, int leftShoulderMotorID, int wristMotorID, int shoulderEncoderID, int wristEncoderID,
               double kPShoulder, double kIShoulder, double kDShoulder,
               double kPWrist, double kIWrist, double kDWrist,
               double kGWrist, double kSWrist, double kVWrist, double kAWrist,
               double kGShoulder, double kSShoulder, double kVShoulder, double kAShoulder,
               double shoulderLength, double wristLength, double shoulderCOMLen, double wristCOMLen,
               double shoulderCOMOffset, double wristCOMOffset, double shoulderMass, double wristMass,
               double shoulderGearing, double wristGearing,
               Rotation2d criticalShoulderAngle, Rotation2d criticalWristAngle,
               Rotation2d highTransitionShoulderAngle, Rotation2d highTransitionWristAngle,
               double maxSpeed, double maxAccel) {

        shoulderMotorLeft = new CANSparkMax(leftShoulderMotorID, kBrushless);
        shoulderMotorRight = new CANSparkMax(rightShoulderMotorID, kBrushless);
        wristMotor = new CANSparkMax(wristMotorID, kBrushless);

        shoulderMotorRight.setIdleMode(CANSparkBase.IdleMode.kBrake);
        shoulderMotorLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);
        wristMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        shoulderMotorLeft.setInverted(true);
        shoulderMotorRight.setInverted(false);
        wristMotor.setInverted(true);

        wristMotor.setSmartCurrentLimit(20);
        shoulderMotorRight.setSmartCurrentLimit(20);
        shoulderMotorLeft.setSmartCurrentLimit(20);

        shoulderMotorRight.follow(shoulderMotorLeft, true);

        shoulderEncoder = new CANCoder(shoulderEncoderID);
        wristEncoder = new CANCoder(wristEncoderID);

        shoulderRelEncoder = shoulderMotorLeft.getEncoder();
        wristRelEncoder = wristMotor.getEncoder();

        this.maxAccel = maxAccel; this.maxSpeed = maxSpeed;
        this.shoulderLength = shoulderLength; this.wristLength = wristLength;
        this.shoulderCOMLen = shoulderCOMLen; this.wristCOMLen = wristCOMLen;
        this.shoulderCOMOffset = shoulderCOMOffset; this.wristCOMOffset = wristCOMOffset;
        this.shoulderMass = shoulderMass; this.wristMass = wristMass;
        this.shoulderGearing = shoulderGearing; this.wristGearing = wristGearing;
        wristFF = new ArmFeedforward(kSWrist, kGWrist, kVWrist, kAWrist);
        shoulderFF = new ArmFeedforward(kSShoulder, kGShoulder, kVShoulder, kAShoulder);

        shoulderController = new PIDController(kPShoulder, kIShoulder, kDShoulder);
        wristController = new PIDController(kPWrist, kIWrist, kDWrist);
        //wristController.setIZone();

        interShoulder = criticalShoulderAngle; interWrist = criticalWristAngle;
        this.highTransitionShoulderAngle = highTransitionShoulderAngle; this.highTransitionWristAngle = highTransitionWristAngle;
		setShoulderDesState(shoulderState().getDegrees());
		setWristDestState(wristState().getDegrees());
        shoulderController.reset();
        wristController.reset();
	    shoulderSysIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, stepVoltage,timeout),
			new SysIdRoutine.Mechanism(
					(Measure<Voltage> volts)->{
						shoulderMotorLeft.setVoltage(volts.in(Volts));
						},
					log -> {
						log.motor("wrist-motor").voltage(
								m_appliedVoltage.mut_replace(
									shoulderMotorLeft.getAppliedOutput()*shoulderMotorLeft.getBusVoltage(), Volts)
						    ).angularPosition(m_angle.mut_replace(COMAngle().getDegrees(),Degrees)
							).angularVelocity(m_velocity.mut_replace(getShoulderVelocity(),DegreesPerSecond));
					},
					this
			)
		);
    }

    public void periodic(){
        //TODO: make the mechanism 2d object in here
        SmartDashboard.putNumber("shoulderValue", shoulderState().getDegrees());
        SmartDashboard.putNumber("wristValue", wristState().getDegrees());
        SmartDashboard.putNumber("shoulderRel", shoulderPosRel());
        SmartDashboard.putNumber("wristRel", wristPosRel());
        SmartDashboard.putNumber("desiredShould", getShoulderDesState());
        SmartDashboard.putNumber("desiredWrist", getWristDesState());
        SmartDashboard.putNumber("combinedWrist", combinedWrist().getDegrees());
        SmartDashboard.putNumber("com angle",COMAngle().getDegrees());
    }

    public void pathFollow(Rotation2d shoulder, Rotation2d wrist, double wristVel, double shoulderVel){
        //x is shoulder, y is wrist
        shoulderController.setSetpoint(shoulder.getDegrees());
        double shoulderPow = (shoulderController.calculate(shoulderState().getDegrees()));

        wristController.setSetpoint(wrist.getDegrees());
        double wristPow = (wristController.calculate(wristState().getDegrees()));
        //if (combWristConversion(shoulder, wrist).getDegrees() >= 80) wristPow = wristPow/2;
//        if (!boundChecker.inBounds(shoulder, wrist, shoulderLength, wristLength)){
//            if (boundChecker.negDerivShoulder(shoulderState(), shoulder, shoulderLength, wristLength)) shoulderPow = 0;
//            if (boundChecker.negDerivWrist(wristState(),wrist, wristLength)) wristPow = 0;
//        }
        shoulderPow += getShoulderFF(shoulder, wrist, Units.degreesToRadians(shoulderVel));
        wristPow += getWristFF(shoulder, wrist, Units.degreesToRadians(wristVel));
        shoulderVoltage(shoulderPow);
        wristVoltage(wristPow);
    }

    public void shoulderVoltage(double power){
        power = Math.max(Math.min(power, 8),-8);
        SmartDashboard.putNumber("shoulderpow", power);
        shoulderMotorLeft.setVoltage(power);
    }
    public void wristVoltage(double power){
        power = Math.max(Math.min(power, 8), -8);
        SmartDashboard.putNumber("wristpow", power);
        wristMotor.setVoltage(power);
    }

    public double getArmSpeed(){
        return 0;
    }
    public double getShoulderFF(Rotation2d shoulderVal, Rotation2d wristVal, double velocity){
        return shoulderFF.calculate(COMAngleConversion(shoulderVal, wristVal).getRadians(), velocity);
    }
    public double getWristFF(Rotation2d shoulderVal, Rotation2d wristVal, double velocity){
        return wristFF.calculate(combWristConversion(shoulderVal, wristVal).getRadians(), velocity);
    }

    public Command goToPoint(Rotation2d shoulderPos, Rotation2d wristPos) {

        SmartDashboard.putNumber("movingToPoint", shoulderPos.getDegrees());
        Rotation2d safeShoulder, safeWrist;
        if ((shoulderState().getDegrees() < interShoulder.getDegrees() && shoulderPos.getDegrees() < interShoulder.getDegrees()) ||
                (shoulderState().getDegrees() > interShoulder.getDegrees() && shoulderPos.getDegrees() > interShoulder.getDegrees())) {
            SmartDashboard.putBoolean("inside convex region 1 or 2", true);
            return new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel).withName("Arm to dest");
        }
        safeShoulder = interShoulder; safeWrist = interWrist;
        SmartDashboard.putBoolean("transition spot", true);

        //this is if we have a bumper intersection location
        if (shoulderLength*Math.sin(shoulderState().getRadians()-Math.PI/2) - wristLength*Math.sin(combinedWrist().getRadians()) + Units.inchesToMeters(14.5)
         <= Units.inchesToMeters(10) &&
        shoulderLength*Math.sin(shoulderState().getRadians()-Math.PI/2) - wristLength*Math.sin(combinedWrist().getRadians()) + Units.inchesToMeters(14.5)
                >= Units.inchesToMeters(5)
        ||
        shoulderLength*Math.sin(shoulderPos.getRadians()-Math.PI/2) - wristLength*Math.sin(combWristConversion(shoulderPos, wristPos).getRadians()) + Units.inchesToMeters(14.5)
        <= Units.inchesToMeters(10) &&
        shoulderLength*Math.sin(shoulderPos.getRadians()-Math.PI/2) - wristLength*Math.sin(combWristConversion(shoulderPos, wristPos).getRadians()) + Units.inchesToMeters(14.5)
                >= Units.inchesToMeters(5)

        || wristPos.getDegrees() <= -90 || wristState().getDegrees() <= -90
        ){
            System.out.println("new motion!");
            return new SequentialCommandGroup(
                    new ArmPathFollow(this, highTransitionShoulderAngle, highTransitionWristAngle, maxSpeed, maxAccel),
                    new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel)
            );
        }
        //otherwise just go through safe point
        System.out.println("normal safe point");
        return new SequentialCommandGroup(
                new ArmPathFollow(this, safeShoulder, safeWrist, maxSpeed, maxAccel).withName("Arm to safe"),
                new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel).withName("Arm safe -> dest")
        );
    }

    public Translation2d autoAim(Supplier<Pose2d> robotPos){
        double dist = AllianceFlip.apply(UsefulPoints.Points.middleOfSpeaker).getDistance(robotPos.get().getTranslation());
        dist = Units.metersToInches(dist);
        double func;
        if (-.901*dist+130.46 < -12.0){
            func = -38.5+Math.pow(.901*dist-130.46, 1.0/3.0);
        }
        else if (-.901*dist+130.46 > 12.0){
            func = -38.5-Math.pow(-.901*dist+130.46, 1.0/3.0);
        }
        else{
            func = (-36.29+40.71)/24*(dist-144.8)-38.5;
        }
        func -= 14.5;
        return new Translation2d(120, Math.min(Math.max(-60, func), -45));

    }


    public Command controlRobot2O(Rotation2d shoulderPos, Rotation2d wristPos){
        if (boundChecker.inPointyPart2O(shoulderState(), wristState())
        || boundChecker.inPointyPart2O(shoulderPos, wristPos)){
            return new SequentialCommandGroup(
                    new ArmPathFollow(this, interShoulder, interWrist, maxSpeed, maxAccel),
                    new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel)
            );
        }

        return new ArmPathFollow(this, shoulderPos, wristPos, maxSpeed, maxAccel);
    }

    public void stopMotors(){
        wristVoltage(0); shoulderVoltage(0);
    }


    public Rotation2d shoulderState() {
        double deg = shoulderEncoder.getAbsolutePosition()+shoulderOffset;
        if (deg < -180) deg = deg + 360;
        if (deg > 180) deg = deg - 360;
        return Rotation2d.fromDegrees(deg);
    }

    public Rotation2d wristState(){
        //return Rotation2d.fromDegrees(wristPosRel());
        double deg = wristEncoder.getAbsolutePosition() + wristOffset;
        if (deg < -180) deg = deg + 360;
        if (deg > 180) deg = deg - 360;
        return Rotation2d.fromDegrees(-deg);
    }

    public Rotation2d combinedWrist(){
        return combWristConversion(shoulderState(), wristState());
    }
    public Rotation2d combWristConversion(Rotation2d shoulder, Rotation2d wrist){
        return Rotation2d.fromDegrees(180-(shoulder.getDegrees()-90+180+wrist.getDegrees()));
    }

    public Rotation2d COMAngle(){
        return COMAngleConversion(shoulderState(), wristState());
    }

    public Rotation2d COMAngleConversion(Rotation2d shoulder, Rotation2d wrist){
        double shoulderDeg = shoulder.getDegrees();//Math.round(shoulder);//Math.floor(shoulder) + Math.round((shoulder-Math.floor(shoulder))*10)/10.0;
        double wristDeg = wrist.getDegrees();//Math.round(wrist); //+ Math.round((wrist-Math.floor(wrist))*10)/10.0;
        double COMx = (shoulderMass*shoulderCOMLen*Math.cos(Units.degreesToRadians(shoulderDeg-90+shoulderCOMOffset))
                + wristMass*(shoulderLength*Math.cos(Units.degreesToRadians(shoulderDeg-90))
                +wristCOMLen*Math.cos(Units.degreesToRadians(wristDeg-wristCOMOffset))))/(shoulderMass+wristMass);
        double COMy = (shoulderMass*shoulderCOMLen*Math.sin(Units.degreesToRadians(shoulderDeg-90+shoulderCOMOffset))
                + wristMass*(shoulderLength*Math.sin(Units.degreesToRadians(shoulderDeg-90))
                -wristCOMLen*Math.sin(Units.degreesToRadians(wristDeg-wristCOMOffset))))/(shoulderMass+wristMass);
        return Rotation2d.fromRadians(Math.atan2(COMy,COMx));
    }

    public double shoulderPosRel(){
        return shoulderRelEncoder.getPosition();
    }
    public double wristPosRel(){
        double val = (wristRelEncoder.getPosition()*(-126.3+2)/(-0.548-15.8))*22/48-126.3+2;
        return val;
    }

    public void shoulderVoltageController(double shoulderPow){
        shoulderVoltage(shoulderPow);
        setShoulderDesState(shoulderState().getDegrees());
    }
    public void wristVoltageController(double wristPow){
        wristVoltage(wristPow);
        setWristDestState(wristState().getDegrees());
    }
    public void setShoulderDesState(double pos){
        currShoulder = pos;
    }
    public void setWristDestState(double pos){
        currWrist = pos;
    }

    public void setState(double shoulderDesState, double wristDesState){
        setShoulderDesState(shoulderDesState); setWristDestState(wristDesState);
    }
    public double getShoulderDesState(){
        return currShoulder;
    }
    public double getWristDesState(){
        return currWrist;
    }

    public double getWristVelocity(){
        return wristRelEncoder.getVelocity()*(-126.3+2)/(-0.548-15.8)*22/48;
    }
    public double getShoulderVelocity(){
        return shoulderEncoder.getVelocity();
    }

    public void holdPos(double shoulder, double wrist){
        SmartDashboard.putNumber("shoulderRelSetpt", shoulder);
        SmartDashboard.putNumber("wristRelSetpt", wrist);
        wristController.setSetpoint(wrist);
        shoulderController.setSetpoint(shoulder);
        Rotation2d wristRel = Rotation2d.fromDegrees(wrist);
        Rotation2d shoulderRel = Rotation2d.fromDegrees(shoulder);
        double wristPow = wristController.calculate(wristState().getDegrees());
        //if (combWristConversion(Rotation2d.fromDegrees(shoulder), Rotation2d.fromDegrees(wrist)).getDegrees() >= 80) wristPow /= 2;
        wristVoltage(wristPow+getWristFF(shoulderRel, wristRel, 0));
        shoulderVoltage(shoulderController.calculate(shoulderState().getDegrees())+getShoulderFF(shoulderRel, wristRel, 0));
        //wristPower(0);
        //shoulderPower(0);
    }

	public Command shoulderQuasiStatic(SysIdRoutine.Direction direction){
		return shoulderSysIdRoutine.quasistatic(direction).handleInterrupt(()->setShoulderDesState(shoulderState().getDegrees())).andThen(()->setShoulderDesState(shoulderState().getDegrees()));
	}

	public Command shoulderDynamic(SysIdRoutine.Direction direction){
		return shoulderSysIdRoutine.dynamic(direction).handleInterrupt(()->setShoulderDesState(shoulderState().getDegrees())).andThen(()->setShoulderDesState(shoulderState().getDegrees()));
	}

}