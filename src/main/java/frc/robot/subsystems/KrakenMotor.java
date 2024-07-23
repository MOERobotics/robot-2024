package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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
	public KrakenMotor(int motorID) {
		this.motor = new TalonFX(motorID);
		TalonFXConfiguration MotorConfig = new TalonFXConfiguration();
		MotorConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
		MotorConfig.CurrentLimits.withStatorCurrentLimit(60);
		motor.getConfigurator().apply(MotorConfig);
		motor.setNeutralMode(NeutralModeValue.Brake);
	}
	public KrakenMotor(int motorID, boolean invert, double motorP, double motorI, double motorD, double motorFF) {
		this.motor = new TalonFX(motorID);
		TalonFXConfiguration MotorConfig = new TalonFXConfiguration();
		MotorConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
		MotorConfig.CurrentLimits.withStatorCurrentLimit(60);
		MotorConfig.MotorOutput.Inverted = invert?InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive;
		motor.getConfigurator().apply(MotorConfig);
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
		Slot0Configs PIDConfigs = new Slot0Configs();
		PIDConfigs.kP = motorP;
		PIDConfigs.kI = motorI;
		PIDConfigs.kD = motorD;
		motor.getConfigurator().apply(PIDConfigs);
		driveVelocityVoltage = new VelocityVoltage(0).withSlot(0);
		this.motorFF = motorFF;
	}

	@Override
	public void setInvert(boolean invert) {
		motor.getConfigurator().apply(
				new TalonFXConfiguration().withMotorOutput(
						new MotorOutputConfigs().withInverted(
								invert?InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive
						)
				)
		);
	}

	@Override
	public double getMotorPosition(){
		return motor.getPosition().getValue();
	}

	@Override
	public double getMotorVelocity(){
		return motor.getVelocity().getValue()/60;
	}

	@Override
	public void set(double power){
		motor.set(power);
	}

	@Override
	public double get() {
		return motor.get();
	}

	/**Set drive motor velocity in rotations per second
	 * @param vel Velocity in rotations per minute
	 */
	@Override
	public void setMotorVelocity(double vel){
		motor.setControl(driveVelocityVoltage.withVelocity(vel*60).withFeedForward(motorFF));
	}

	@Override
	public void stopMotor(){
		motor.stopMotor();
	}

	@Override
	public int getDeviceID(){
		return motor.getDeviceID();
	}

}
