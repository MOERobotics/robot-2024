package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.List;

public class Vision {

    Pose2d odometryPosition;
    Transform3d cameraPosition;

    Pose2d robotPosition;

    public void setOdometryPosition(Pose2d fieldToOdometry){ //Rohan
        odometryPosition = fieldToOdometry;
    }

    public Pose2d getRobotPosition(){// Tanmaybe
        //transform = objectdetection.getTransform();
        //robotPosition = odometryPosition+transform;
        return robotPosition;
    }

    public void setCameraPosition(Transform3d robotToCamera){ //Rohan
        cameraPosition = robotToCamera;
    }

    public List<ObjectDetection> objectDetection(){ //Both
        return null;
    }
}
