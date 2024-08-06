package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenMotor extends SubsystemBase implements GenericMotor{
	private final TalonFX motor;
	private VelocityVoltage driveVelocityVoltage;
	private double motorFF;
	private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
	public KrakenMotor(int motorID) {
		this.motor = new TalonFX(motorID);
		motorConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
		motorConfig.CurrentLimits.withStatorCurrentLimit(60);
		motor.getConfigurator().apply(motorConfig);
		motor.setNeutralMode(NeutralModeValue.Brake);
	}
	public KrakenMotor(int motorID, boolean invert, double motorP, double motorI, double motorD, double motorFF) {
		this.motor = new TalonFX(motorID);
		motorConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
		motorConfig.CurrentLimits.withStatorCurrentLimit(60);
		motorConfig.MotorOutput.Inverted = invert?InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive;
		motor.getConfigurator().apply(motorConfig);
		motor.setNeutralMode(NeutralModeValue.Brake);
		Slot0Configs PIDConfigs = new Slot0Configs();
		PIDConfigs.kP = motorP;
		PIDConfigs.kI = motorI;
		PIDConfigs.kD = motorD;
		motor.getConfigurator().apply(PIDConfigs);
		driveVelocityVoltage = new VelocityVoltage(0).withSlot(0);
		this.motorFF = motorFF;
	}

	@Override
	public void setPID(double motorP, double motorI, double motorD, double motorFF){
		Slot0Configs pidConfigs = new Slot0Configs();
		motor.getConfigurator().refresh(pidConfigs);
		pidConfigs.kP = motorP;
		pidConfigs.kI = motorI;
		pidConfigs.kD = motorD;
		motor.getConfigurator().apply(pidConfigs);
		driveVelocityVoltage = new VelocityVoltage(0).withSlot(0);
		this.motorFF = motorFF;
	}

	@Override
	public void setInvert(boolean invert) {
		motor.setInverted(invert);
	}

	@Override
	public double getMotorPosition(){
		return motor.getPosition().getValue();
	}

	@Override
	public double getMotorVelocity(){
		return motor.getVelocity().getValue()*60;
	}

	@Override
	public void set(double power){
		motor.set(power);
	}

	@Override
	public double get() {
		return motor.get();
	}

	@Override
	public void setMotorVelocity(double vel){
		motor.setControl(driveVelocityVoltage.withVelocity(vel/60).withFeedForward(motorFF));
	}

	@Override
	public void stopMotor(){
		motor.stopMotor();
	}

	@Override
	public int getDeviceID(){
		return motor.getDeviceID();
	}

	@Override
	public KrakenMotor withPID(double motorP, double motorI, double motorD, double motorFF) {
		this.setPID(motorP,motorI,motorD,motorFF);
		return this;
	}

	@Override
	public KrakenMotor withInvert(boolean invert) {
		this.setInvert(invert);
		return this;
	}

}
