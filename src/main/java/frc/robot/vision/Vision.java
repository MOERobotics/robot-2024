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


        pubFieldToOdometry = networkTable.getStructTopic("Moenet/field_to_odometry", Pose3d.struct).publish();
        pubRobotToCamera = networkTable.getStructTopic("Moenet/robot_to_camera", Transform3d.struct).publish();
        subOdomToRobot = networkTable.getStructTopic("Moenet/odom_to_robot", Transform3d.struct).subscribe(new Transform3d());
        subObjectDetections = networkTable.getStructArrayTopic("Moenet/object_detection", ObjectDetection.struct)
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

    public List<ObjectDetection> objectDetection(){ //Both
        return null;
    }
}
