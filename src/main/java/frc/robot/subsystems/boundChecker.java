package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public class boundChecker {

    public static boolean inBounds(Rotation2d shoulderPos, Rotation2d wristPos, double shoulderLen, double wristLen){
        return shoulderLen*Math.cos(shoulderPos.getRadians()) + wristLen*Math.cos(shoulderPos.getRadians()+wristPos.getRadians()) < 12;
    }

    public static boolean negDerivShoulder(Rotation2d shoulderPos, double shoulderLen, double wristLen){
        return -shoulderLen*Math.sin(shoulderPos.getRadians())-wristLen*Math.sin(shoulderPos.getRadians()) < 0;
    }
    public static boolean negDerivWrist(Rotation2d wristPos, double wristLen){
        return -wristLen*Math.sin(wristPos.getRadians()) < 0;
    }

    public static boolean inPointyPart2O(Rotation2d shoulderPos, Rotation2d wristPos){
        if (wristPos.getDegrees() <= -30 && shoulderPos.getDegrees() >= 113){
            if (shoulderPos.getDegrees() <= 109){
                return true;
            }
        }
        return false;
    }

}
