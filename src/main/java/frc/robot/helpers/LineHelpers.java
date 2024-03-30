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


    public static double getS(double distance, double maxSpeed, double maxAccel, double v_0, double t){
        // using max speed it finds max velocity that the robot should travel
        double maxVel = Math.min(maxSpeed, (-v_0 + Math.sqrt(v_0*v_0 + 2*maxAccel*distance)/2));
        double t_1 = (maxVel-v_0)/maxAccel;
        double t_3 = maxVel/maxAccel;
        double t_2 = Math.max(0,(distance-(.5*maxAccel*t_1*t_1+v_0*t_1-.5*maxAccel*t_3*t_3+maxVel*t_3))/maxVel);

        // it finds time to max which is the time it takes to reach the max speed(beginning of the trapezoid)
        double d_1 = .5*maxAccel*t_1*t_1+v_0*t_1;
        double d_2 = maxVel*t_2;
        double d_3 = maxVel*t_3-.5*maxAccel*t_3*t_3;

        // if the amount of time you want it take is less than the time it takes to max
        if (t <= t_1){
            return .5*maxAccel*t*t+v_0*t;
        }
        else if ((t-t_1) <= t_2){
            return maxVel*(t-t_1)+d_1;
        }
        else if (t-t_1-t_2 <= t_3){
            double offSetT = t - t_1-t_2;
            return maxVel*(offSetT) - .5*maxAccel*offSetT*offSetT +d_1 + d_2;
        }
        else {
            return d_1 + d_2 + d_3;
        }
    }

    public static double getVel(double distance, double maxSpeed, double maxAccel, double v_0, double t){
        double maxVel = Math.min(maxSpeed, (-v_0 + Math.sqrt(v_0*v_0 + 2*maxAccel*distance)/2));
        double t_1 = (maxVel-v_0)/maxAccel;
        double t_3 = maxVel/maxAccel;
        double t_2 = Math.max(0,(distance-(.5*maxAccel*t_1*t_1+v_0*t_1-.5*maxAccel*t_3*t_3+maxVel*t_3))/maxVel);

        // if the amount of time you want it take is less than the time it takes to max
        if (t <= t_1){
            return maxAccel*t+v_0;
        }
        else if ((t-t_1) <= t_2){
            return maxVel;
        }
        else if (t-t_1-t_2 <= t_3){
            double offSetT = t - t_1-t_2;
            return maxVel - maxAccel*offSetT;
        }
        else {
            return 0;
        }
    }


}
