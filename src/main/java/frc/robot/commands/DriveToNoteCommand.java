package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.vision.Vision;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveToNoteCommand extends Command {

    private final Vision vision;
    private final SwerveDrive subsystem;
    private final Supplier<Double> xspdFunction,yspdFunction,turnspdFunction;
    private final DoubleConsumer rumbleCallback;
	private final double maxMPS;

    private Translation2d target = null;
    private int idleLoopCount = 0;
    private final double Latency = 0.25;//Pose latency in seconds

    public DriveToNoteCommand(SwerveDrive subsystem, Vision vision, Supplier<Double> xspeed, Supplier<Double> yspeed,
                              Supplier<Double> turnspeed, double maxMPS) {
        this(subsystem, vision, xspeed,yspeed,turnspeed, null, maxMPS);
    }
    public DriveToNoteCommand(SwerveDrive subsystem, Vision vision, Supplier<Double> xspeed, Supplier<Double> yspeed,
                              Supplier<Double> turnspeed, DoubleConsumer rumbleCallback, double maxMPS){
		xspdFunction = xspeed;
	    yspdFunction = yspeed;
	    turnspdFunction = turnspeed;
        this.vision = vision;
        this.subsystem = subsystem;
        this.rumbleCallback = rumbleCallback;
		this.maxMPS=maxMPS;
        addRequirements(subsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.target = null;
        idleLoopCount = 0;
    }

    private void updateTarget() {
        var robotPose = subsystem.getBufferedPose(Timer.getFPGATimestamp()-Latency);
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
//            idleLoopCount += 1;
//
//            if (idleLoopCount >= 5 && rumbleCallback != null){
//                rumbleCallback.accept(1);
//            }
	        subsystem.driveAtSpeed(xspdFunction.get(), yspdFunction.get(), turnspdFunction.get(), true);
            return;
        }
        // Put the detection on NetworkTables, for debugging
        subsystem.field.getObject("NoteTarget").setPose(new Pose2d(target, new Rotation2d()));
        // Drive towards target
        var robotPose = subsystem.getEstimatedPose();
        var delta = target.minus(robotPose.getTranslation());

        var unitDelta = delta.div(delta.getNorm());//.times(speedSupplier.getAsDouble());

        var robotAngle = unitDelta.getAngle();
        if (delta.getNorm() <= Units.inchesToMeters(24)) robotAngle = robotPose.getRotation().times(1);
		SmartDashboard.putNumber("robot Object Detection angle", robotAngle.getDegrees());
       // var yawOffset = subsystem.getRotation2d().minus(robotPose.getRotation());
        subsystem.setDesiredYaw(robotAngle.getDegrees());//Set absolute heading

        var speedVal = Math.max(0, Math.hypot(xspdFunction.get(), yspdFunction.get())-.05)*(maxMPS);
        if (speedVal < .1) speedVal = 0;

        subsystem.driveAtSpeed(robotAngle.getCos()*speedVal,
                robotAngle.getSin()*speedVal, 0, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
