package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class NeoMotor extends SubsystemBase implements GenericMotor{
	private final CANSparkMax motor;
	private final RelativeEncoder encoder;
	private final SparkPIDController controller;
	public NeoMotor(int motorID) {
		motor = new CANSparkMax(motorID, kBrushless);
		motor.setSmartCurrentLimit(60);
		motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		encoder = motor.getEncoder();
		controller = motor.getPIDController();
	}
	public NeoMotor(int motorID, boolean invert, double motorP, double motorI, double motorD, double motorFF) {
		motor = new CANSparkMax(motorID, kBrushless);
		motor.setSmartCurrentLimit(60);
		motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
		motor.setInverted(invert);
		encoder = motor.getEncoder();
		controller = motor.getPIDController();
		controller.setP(motorP);
		controller.setI(motorI);
		controller.setIZone(0);
		controller.setD(motorD);
		controller.setFF(motorFF);
		controller.setOutputRange(-1, 1);
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
	public void setMotorVelocity(double velocity) {
		controller.setReference(velocity, CANSparkMax.ControlType.kVelocity);
	}

	@Override
	public double getMotorVelocity() {
		return encoder.getVelocity();
	}

	@Override
	public double getMotorPosition() {
		return encoder.getPosition();
	}

	@Override
	public void stopMotor() {
		motor.stopMotor();
	}

	@Override
	public void setPID(double motorP, double motorI, double motorD, double motorFF) {
		controller.setP(motorP);
		controller.setI(motorI);
		controller.setIZone(0);
		controller.setD(motorD);
		controller.setFF(motorFF);
		controller.setOutputRange(-1, 1);
	}

	@Override
	public void setInvert(boolean invert) {
		motor.setInverted(invert);
	}

	@Override
	public int getDeviceID(){
		return motor.getDeviceId();
	}

	@Override
	public NeoMotor withPID(double motorP, double motorI, double motorD, double motorFF){
		this.setPID(motorP,motorI,motorD,motorFF);
		return this;
	}

	@Override
	public NeoMotor withInvert(boolean invert){
		this.setInvert(invert);
		return this;
	}
}
