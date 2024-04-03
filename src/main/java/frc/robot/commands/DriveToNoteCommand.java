package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Vision;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToNoteCommand extends Command {

    private final Vision vision;
    private final SwerveDrive subsystem;
    private final DoubleSupplier speedSupplier;
    private final DoubleConsumer rumbleCallback;

    private Translation2d target = null;
    private int idleLoopCount = 0;

    public DriveToNoteCommand(SwerveDrive subsystem, Vision vision, DoubleSupplier speedSupplier) {
        this(subsystem, vision, speedSupplier, null);
    }
    public DriveToNoteCommand(SwerveDrive subsystem, Vision vision, DoubleSupplier speedSupplier, DoubleConsumer rumbleCallback){
        this.speedSupplier = speedSupplier;
        this.vision = vision;
        this.subsystem = subsystem;
        this.rumbleCallback = rumbleCallback;
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

            if (idleLoopCount >= 5 && rumbleCallback != null){
                rumbleCallback.accept(1);
            }
            return;
        }
        // Drive towards target
        var robotPose = subsystem.getEstimatedPose();
        var delta = target.minus(robotPose.getTranslation());
        var unitDelta = delta.div(delta.getNorm()).times(speedSupplier.getAsDouble());

        var robotAngle = unitDelta.getAngle();
        subsystem.setDesiredYaw(robotAngle.getDegrees());

        //subsystem.driveAtSpeed(unitDelta.getX(), unitDelta.getY(), 0, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (rumbleCallback != null){
            rumbleCallback.accept(0);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
