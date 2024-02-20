package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberArm;
import frc.robot.subsystems.ExampleSubsystem;

public class ClimbTrapCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final ClimberArm climberArmLeft;
    private final ClimberArm climberArmRight;

    private final double speed;



    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ClimbTrapCommand(ClimberArm climberArmRight, ClimberArm climberArmLeft, double speed) {
        this.climberArmRight = climberArmRight;
        this.climberArmLeft = climberArmLeft;
        this.speed = speed;


        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climberArmLeft, climberArmRight);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double finalSpeed = 0;

        // If both arms can go up it goes up
      /*  if (climberArmLeft.canGoUp() && climberArmRight.canGoUp()) {
            finalSpeed = speed;
        } else {
            if (climberArmLeft.hasChainTop() && climberArmRight.hasChainTop()) {
                finalSpeed = 0;
            } else if (climberArmLeft.hasChainBottom() && climberArmRight.hasChainBottom()) {
                finalSpeed = 0;
            } else {
                if (climberArmLeft.canGoUp() || climberArmRight.canGoUp()) {
                    finalSpeed = speed;
                }
                // Otherwise, go down until hitting the chains
                else {
                    if(climberArmLeft.canGoDown() && climberArmRight.canGoDown()){
                        finalSpeed = -speed;
                    } else{
                        finalSpeed=0;
                    }

                }
            }
        }


       */

        climberArmLeft.driveRight(finalSpeed);
        climberArmRight.driveLeft(finalSpeed);

        SmartDashboard.putNumber("climber speed", finalSpeed);


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
