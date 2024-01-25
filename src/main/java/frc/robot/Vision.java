package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.List;

public class Vision {

    Pose2d odometryPosition;

    public void setOdometryPosition(Pose2d pose){ //Rohan
        odometryPosition = pose;
    }

    public Pose2d getRobotPosition(){// Tanmaybe
        return null;
    }

    public void setCameraPosition(){ //Rohan

    }

    public List<Pose2d> objectDetection(){ //Both
        return null;
    }
}
