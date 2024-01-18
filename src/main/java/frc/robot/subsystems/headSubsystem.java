// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class headSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public headSubsystem() {
    //instantiate shooter motors, encoders, sensors, PID
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
  public boolean hasNote(){
    return false;
  }

  public void setShooterTop(){

  }
  public void setShooterBottom(){

  }
  public double getShooterSpeedTop(){
    return 0;
  }
  public double getShooterSpeedBottom(){
    return 0;
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
