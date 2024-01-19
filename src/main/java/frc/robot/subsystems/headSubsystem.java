// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class headSubsystem extends SubsystemBase {
  private final CANSparkMax collectorMotorTop = new CANSparkMax(24, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax collectorMotorTop2 = new CANSparkMax(23, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax collectorMotorBot = new CANSparkMax(25, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkMax collectorMotorBot2 = new CANSparkMax(22, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder topEncoder = collectorMotorTop.getEncoder();
  private final RelativeEncoder bottomEncoder = collectorMotorTop.getEncoder();
  private final DigitalInput collectorBeam = new DigitalInput(1);

  public headSubsystem() {


  }


  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  //Has a note
  public boolean hasNote(){
    return collectorBeam.get();
  }

  public void setShooterTop(double speed){
        collectorMotorTop.set(speed);
        collectorMotorTop2.set(speed);

  }
  public void setShooterBottom(double speed){
      collectorMotorBot.set(speed);
      collectorMotorBot2.set(speed);

  }
  public double getShooterSpeedTop(){
    return topEncoder.getVelocity();
  }
  public double getShooterSpeedBottom(){
      return bottomEncoder.getVelocity();
  }

  //Within reasonable range to shoot?
  public boolean inRange() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  //Shooter april tag seen
  public boolean seeSpeaker() {
    // Query some boolean state, such as a digital sensor.
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


  //Good to shoot
  public boolean readyShoot() {
    //if motors up to speed
    //if aimed
    //if see speaker

    // Query some boolean state, such as a digital sensor.
    return false;
  }

  //Shoot the note
  public void shoot(){
    //Feed note into shooter
  }

  //Set the desired motor speed
  public void setShooterSpeed(double shooterSpeed){

  }

  //Stop motors
  public void stop(){
    collectorMotorTop.stopMotor();
    collectorMotorBot.stopMotor();
    collectorMotorBot2.stopMotor();
    collectorMotorTop2.stopMotor();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
