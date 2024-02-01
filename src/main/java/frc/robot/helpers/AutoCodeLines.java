package frc.robot.helpers;

import org.opencv.core.Point;

public class AutoCodeLines {


    public static double getDistance(Point p, Point q){
        return Math.sqrt((q.x-p.x)*(q.x-p.x) + (q.y-p.y)*(q.y-p.y));
    }

    public static double getSlopeX(Point p, Point q){
        return (q.x - p.x)/getDistance(p,q);
    }

    public static double getSlopeY(Point p, Point q){
        return (q.y - p.y)/getDistance(p,q);
    }

    public static double getPositionX(Point p, Point q, double s){
        return (getSlopeX(p,q)*(s) + p.x);
    }

    public static double getPositionY(Point p, Point q, double s){
        return (getSlopeY(p,q)*(s) + p.y);
    }

    public static double getVelocityX(Point p, Point q, double s){
        return getSlopeX(p,q);
    }

    public static double getVelocityY(Point p, Point q, double s){
        return getSlopeY(p,q);
    }

    public static double getdS(double distance, double timeToMax, double averageSpeed, double t){
        double overAllTime = distance/averageSpeed;
        double minSpeed = 5;
        double maxSpeed = (distance-minSpeed*timeToMax)/(overAllTime-timeToMax);
        if (t < timeToMax){
            return minSpeed + ((maxSpeed-minSpeed)/(timeToMax))*t;
        }
        else if (t < overAllTime-timeToMax){
            return maxSpeed;
        }
        else if (t <= overAllTime){
            return maxSpeed - ((maxSpeed-minSpeed)/(timeToMax))*(t-overAllTime+timeToMax);
        }
        else{
            return minSpeed;
        }
    }


    public static double getS(double distance, double timeToMax, double averageSpeed, double t){
        double overAllTime = distance/averageSpeed;
        double minSpeed = 5;
        double maxSpeed = (distance-minSpeed*timeToMax)/(overAllTime-timeToMax);
        double currSpeed = getdS(distance,timeToMax,averageSpeed,t);
        if (t < timeToMax){
            return (currSpeed+minSpeed)*t/2;
        }
        else if (t < overAllTime-timeToMax){
            return (minSpeed + maxSpeed)*timeToMax/2 + currSpeed*(t-timeToMax);
        }
        else if (t <= overAllTime){
            return distance - (currSpeed+minSpeed)*(overAllTime-t)/2;
        }
        else{
            return distance;
        }
    }


}
