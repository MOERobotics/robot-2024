package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

public class AllianceFlip {
	public static double fieldWidth = 16.541052;//field width in meters
	public static boolean shouldFlip(){
		var alliance = DriverStation.getAlliance();
		if (alliance.isEmpty()) {
			DriverStation.reportError("No alliance color", false);
			return false;
		}
		return alliance.get()==DriverStation.Alliance.Red;
	}
	public static Translation2d apply(Translation2d init){
		if(shouldFlip()){
			return new Translation2d(fieldWidth-init.getX(),init.getY());
		}
		return init;
	}

	public static Rotation2d apply(Rotation2d init){
		if(shouldFlip()){
			return new Rotation2d(-init.getCos(),init.getSin());
		}
		return init;
	}

	public static Pose2d apply(Pose2d init){
		if(shouldFlip()){
			Translation2d flippedTranslation= apply(init.getTranslation());
			Rotation2d flippedRotation = apply(init.getRotation());
			return new Pose2d(flippedTranslation,flippedRotation);
		}
		return init;
	}

	public static List<Translation2d> apply(List<Translation2d>pts){
		return pts.stream().map(AllianceFlip::apply).toList();
	}

}
