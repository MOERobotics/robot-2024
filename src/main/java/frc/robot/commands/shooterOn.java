package frc.robot.commands;

import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.ExampleSubsystem;

public class shooterOn {

    private Solenoid shooter;

    public shooterOn(Solenoid shooter) {
        this.shooter = shooter;



    }


    public void execute() {
        shooter.set(true);

    }


}
