package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmToState extends Command {

    private final Arm.State desState;
    private final Arm arm;

    public MoveArmToState(Arm arm, double x, double y){
        this.arm = arm;
        desState = arm.armInverseKinematics(x,y);
        addRequirements(arm);
    }

    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
