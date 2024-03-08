package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CollectorFinger extends SubsystemBase {

	private final Servo trapServo;
	public CollectorFinger(int servoID) {
		trapServo = new Servo(servoID);
		trapServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
//		trapServo.setPosition(0);
	}

	//sets position of servo
	public void setServoPos(double pos){
		trapServo.setPosition(pos);
	}

	public void stopServo(){
		trapServo.setDisabled();
	}

	@Override
	public void periodic(){
		SmartDashboard.putNumber("Desired Position: ", trapServo.get());
	}
}
