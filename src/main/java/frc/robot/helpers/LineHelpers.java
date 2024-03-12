package frc.robot.helpers;


import edu.wpi.first.math.geometry.Translation2d;

public class LineHelpers {

    public static double getSlopeX(Translation2d p, Translation2d q){
        return (q.getX() - p.getX())/p.getDistance(q);
    }

    public static double getSlopeY(Translation2d p, Translation2d q){
        return (q.getY() - p.getY())/p.getDistance(q);
    }

    public static double getPositionX(Translation2d start, Translation2d q, double s){
        return (getSlopeX(start,q)*(s) + start.getX());
    }

    public static double getPositionY(Translation2d p, Translation2d q, double s){
        return (getSlopeY(p,q)*(s) + p.getY());
    }



public static Translation2d getPosition (Translation2d start, Translation2d end, double s){
       return start.interpolate(end, s/ start.getDistance(end));
}


    public static double getS(double distance, double maxSpeed, double maxAccel, double t){
        // using max speed it finds max velocity that the robot should travel
        double maxVel = Math.min(maxSpeed, Math.sqrt(distance*maxAccel));
        // it finds time to max which is the time it takes to reach the max speed(beginning of the trapezoid)
        double timeToMax = maxVel/maxAccel;
        // calculates overlap time between ??
        double overLapTime = (distance-maxVel*timeToMax);

        // if the amount of time you want it take is less than the time it takes to max
        if (t < timeToMax){

            // returns s
            return .5*maxAccel*t*t;
        }

        // if the robot is speeding up?

        else if (t <= overLapTime+timeToMax){

            // return s
            return .5*maxAccel*timeToMax*timeToMax + maxVel*(t-timeToMax);


        }
        //
        else if (t < overLapTime+2*timeToMax){
            double prevDist = .5*maxAccel*timeToMax*timeToMax + maxVel*overLapTime;
            return prevDist - .5*maxAccel*t*t + maxVel*t;
        }
        else{
            return distance;
        }
    }


}
