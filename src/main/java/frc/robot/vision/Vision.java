package frc.robot.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;

import java.util.List;

public class Vision {
    Transform3d cameraPosition;

    StructPublisher<Pose3d> pubFieldToOdometry;
    StructPublisher<Transform3d> pubRobotToCamera;

    StructSubscriber<Transform3d> subOdomToRobot;

    StructArraySubscriber<ObjectDetection> subObjectDetections;

    Pose3d fieldToOdometry3d = new Pose3d();
    Transform3d odometryCorrection = new Transform3d();

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

    private Transform3d getOdometryCorrection(){           //Added in timestamps to show whether subOdomToRobot has data.
        if (subOdomToRobot.getAtomic().timestamp == 0){
            return odometryCorrection;
        } else {
            return subOdomToRobot.get();
        }
    }

    protected Pose3d getRobotPosition3d() {
        return fieldToOdometry3d.transformBy(getOdometryCorrection());
    }

    public Pose2d getRobotPosition(){// Tanmaybe
        return getRobotPosition3d().toPose2d();
        //translate pose from camera to robot itself


    }

    public void setRobotPosition(Pose2d odometry, Pose2d pose){
        setOdometryPosition(odometry);
        Transform2d offSet = pose.minus(odometry);
        odometryCorrection = new Transform3d(offSet.getX(), offSet.getY(), 0, new Rotation3d(0, 0, offSet.getRotation().getRadians()));
    }

    public void setCameraPosition(Transform3d robotToCamera){ //Rohan
        pubRobotToCamera.set(robotToCamera);
    }

    public ObjectDetection[] objectDetection(){ //Both
        return subObjectDetections.get();
    }
}