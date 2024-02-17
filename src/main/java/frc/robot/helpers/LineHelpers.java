package frc.robot.helpers;


import edu.wpi.first.math.geometry.Translation2d;

public class LineHelpers {

    public static double getSlopeX(Translation2d p, Translation2d q){
        return (q.getX() - p.getX())/p.getDistance(q);
    }

    public static double getSlopeY(Translation2d p, Translation2d q){
        return (q.getY() - p.getY())/p.getDistance(q);
    }

    public static double getPositionX(Translation2d p, Translation2d q, double s){
        return (getSlopeX(p,q)*(s) + p.getX());
    }

    public static double getPositionY(Translation2d p, Translation2d q, double s){
        return (getSlopeY(p,q)*(s) + p.getY());
    }


    public static double getS(double distance, double maxSpeed, double maxAccel, double t){
        double maxVel = Math.min(maxSpeed, Math.sqrt(distance*maxAccel));
        double timeToMax = maxVel/maxAccel;
        double overLapTime = (distance-maxVel*timeToMax);
        if (t < timeToMax){
            return .5*maxAccel*t*t;
        }
        else if (t <= overLapTime+timeToMax){
            return .5*maxAccel*timeToMax*timeToMax + maxVel*(t-timeToMax);
        }
        else if (t < overLapTime+2*timeToMax){
            double prevDist = .5*maxAccel*timeToMax*timeToMax + maxVel*overLapTime;
            return prevDist - .5*maxAccel*t*t + maxVel*t;
        }
        else{
            return distance;
        }
    }


}
