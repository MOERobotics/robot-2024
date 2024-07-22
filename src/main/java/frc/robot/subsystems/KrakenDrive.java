package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenDrive extends SubsystemBase {
	private final TalonFX driveMotor;
	private final VelocityVoltage driveVelocityVoltage;
	private final double driveFF;
	public KrakenDrive(int driveMotorID, double driveP, double driveI, double driveD, double driveFF) {
		this.driveMotor = new TalonFX(driveMotorID);
		TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
		driveMotorConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
		driveMotorConfig.CurrentLimits.withStatorCurrentLimit(60);
		driveMotor.getConfigurator().apply(driveMotorConfig);
		driveMotor.setNeutralMode(NeutralModeValue.Brake);
		Slot0Configs PIDConfigs = new Slot0Configs();
		PIDConfigs.kP = driveP;
		PIDConfigs.kI = driveI;
		PIDConfigs.kD = driveD;
		driveMotor.getConfigurator().apply(PIDConfigs);
		driveVelocityVoltage = new VelocityVoltage(0).withSlot(0);
		this.driveFF = driveFF;
	}

	public double getDriveRot(){
		return driveMotor.getPosition().getValue();
	}

	public double getDriveVelocity(){
		return driveMotor.getVelocity().getValue();
	}

	public void setDriveState(double Vel){
		driveMotor.set(Vel);
	}

	/**Set drive motor velocity in rotations per second
	 * @param Vel Velocity in rotations per second
	 */
	public void setDriveVelocity(double Vel){
		driveMotor.setControl(driveVelocityVoltage.withVelocity(Vel).withFeedForward(driveFF));
	}

	public void stopMotor(){
		driveMotor.stopMotor();
	}

}
