package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Vision;


public class AutoDriveToNoteCommand extends Command {

    private final Vision vision;
    private final SwerveDrive subsystem;

    private Translation2d target = null;
    private int idleLoopCount = 0;
    private double speed;
    private double speedMultiplier;
    public AutoDriveToNoteCommand(SwerveDrive subsystem, Vision vision, double SpeedMultiplier){
        this.vision = vision;
        this.subsystem = subsystem;
        this.speedMultiplier=SpeedMultiplier;
        addRequirements(subsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.target = null;
        idleLoopCount = 0;
    }

    private void updateTarget() {
        var robotPose = subsystem.getEstimatedPose();
        var detectionMaxThreshold = Double.POSITIVE_INFINITY;
        if (target != null) {
            detectionMaxThreshold = robotPose.getTranslation().getDistance(target) + Units.feetToMeters(2);
        }

        var detections = vision.detections();
        for (var detection : detections){
            var distance = detection.getNorm();
            if (distance < detectionMaxThreshold){
                var detectionFieldCoord = robotPose.transformBy(new Transform2d(detection, new Rotation2d())).getTranslation();
                target = detectionFieldCoord;
                detectionMaxThreshold = distance;
            }
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        updateTarget();
        if (target == null){
            idleLoopCount += 1;
            return;
        }
        // Put the detection on NetworkTables, for debugging
        subsystem.field.getObject("NoteTarget").setPose(new Pose2d(target, new Rotation2d()));
        // Drive towards target
        var robotPose = subsystem.getEstimatedPose();
        var delta = target.minus(robotPose.getTranslation());
        speed = speedMultiplier*target.getDistance(robotPose.getTranslation())/Units.inchesToMeters(12);
        var unitDelta = delta.div(delta.getNorm()).times(speed);
        var robotAngle = unitDelta.getAngle();
        subsystem.setDesiredYaw(robotAngle.getDegrees());
        subsystem.driveAtSpeed(unitDelta.getX(), unitDelta.getY(), 0, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
