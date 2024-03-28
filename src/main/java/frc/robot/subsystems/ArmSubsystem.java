package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.Supplier;

public class ArmSubsystem extends SubsystemBase {
	public void periodic() {

		//TODO: make the mechanism 2d object in here
		SmartDashboard.putNumber("shoulderValue", shoulderState().getDegrees());
		SmartDashboard.putNumber("wristValue", wristState().getDegrees());
		SmartDashboard.putNumber("shoulderRel", shoulderPosRel());
		SmartDashboard.putNumber("wristRel", wristPosRel());
		SmartDashboard.putNumber("desiredShould", getShoulderDesState());
		SmartDashboard.putNumber("desiredWrist", getWristDesState());
	}

	 public void pathFollow(Rotation2d shoulder, Rotation2d wrist){}

	 public void shoulderPower(double power){};

	 public void wristPower(double power){};

	 public Command goToPoint(Rotation2d shoulderPos, Rotation2d wristPos){return Commands.none();};

	 public Translation2d autoAim(Supplier<Pose2d> robotPos){return new Translation2d();};

	 public Command controlRobot2O(Rotation2d shoulderPos, Rotation2d wristPos){return Commands.none();};

	 public void stopMotors(){};

	 public Rotation2d shoulderState(){return new Rotation2d();};

	 public Rotation2d wristState(){return new Rotation2d();};

	 public double shoulderPosRel(){return 0;};

	 public double wristPosRel(){return 0;};

	 public void shoulderPowerController(double shoulderPow){};

	 public void wristPowerController(double wristPow){};

	 public void setShoulderDesState(double pos){};

	 public void setWristDestState(double pos){};

	 public void setState(double shoulderDesState, double wristDesState){};

	 public double getShoulderDesState(){return 0;};

	 public double getWristDesState(){return 0;};

	 public void holdPos(double shoulderRel, double wristRel){};
}
