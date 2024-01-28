package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;

import java.util.List;

public class Vision {

    Pose2d odometryPosition;
    Transform3d cameraPosition;

    Pose2d robotPosition;
    StructPublisher<Pose3d> pubFieldToOdometry;
    StructPublisher<Transform3d> pubRobotToCamera;

    StructSubscriber<Transform3d> subOdomToRobot;

    StructArraySubscriber<ObjectDetection> subObjectDetections;

    Pose3d fieldToOdometry3d = new Pose3d();

    public Vision() {
        var networkTable = NetworkTableInstance.getDefault();
        pubFieldToOdometry = networkTable.getStructTopic("moenet/tf_field_odom", Pose3d.struct).publish();
        pubRobotToCamera = networkTable.getStructTopic("moenet/robot_to_camera", Transform3d.struct).publish();
        subOdomToRobot = networkTable.getStructTopic("moenet/tf_odom_robot", Transform3d.struct).subscribe(new Transform3d());
        subObjectDetections = networkTable.getStructArrayTopic("moenet/object_detection", ObjectDetection.struct).subscribe(new ObjectDetection[]{});
    }

    public void setOdometryPosition(Pose2d fieldToOdometry){ //Rohan
        Rotation3d fieldToOdometryRotation = new Rotation3d(0,0,fieldToOdometry.getRotation().getRadians());
        fieldToOdometry3d = new Pose3d(fieldToOdometry.getX(),fieldToOdometry.getY(),0,fieldToOdometryRotation);
        pubFieldToOdometry.set(fieldToOdometry3d);
    }

    public Pose2d getRobotPosition(){// Tanmaybe
        return fieldToOdometry3d.transformBy(subOdomToRobot.get()).toPose2d();

    }

    public void setCameraPosition(Transform3d robotToCamera){ //Rohan
        pubRobotToCamera.set(robotToCamera);
    }

    public ObjectDetection[] objectDetection(){ //Both
        return subObjectDetections.get();
    }
}
