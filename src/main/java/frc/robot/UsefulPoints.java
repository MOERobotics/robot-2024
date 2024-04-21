// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class UsefulPoints {
    public static class Points {
        public static Translation2d middleOfSpeaker = new Translation2d(
                0,Units.inchesToMeters(219.5));
        public static Translation2d WingedNote1 = new Translation2d(Units.inchesToMeters(113.6), Units.inchesToMeters(275.6));
        public static Translation2d WingedNote2 = new Translation2d(Units.inchesToMeters(113.6), Units.inchesToMeters(218.6));
        public static Translation2d WingedNote3 = new Translation2d(Units.inchesToMeters(113.6),Units.inchesToMeters(161.6));

        public static Translation2d CenterNote1 = new Translation2d(Units.inchesToMeters(325.6), Units.inchesToMeters(293.6));
        public static Translation2d CenterNote2 = new Translation2d(Units.inchesToMeters(325.6), Units.inchesToMeters(227.6));
        public static Translation2d CenterNote3 = new Translation2d(Units.inchesToMeters(325.6), Units.inchesToMeters(161.6));
        public static Translation2d CenterNote4 = new Translation2d(Units.inchesToMeters(325.6), Units.inchesToMeters(95.6));
        public static Translation2d CenterNote5 = new Translation2d(Units.inchesToMeters(325.6), Units.inchesToMeters(29.6));

        public static Translation2d StartingPointA = new Translation2d(Units.inchesToMeters(18.6), Units.inchesToMeters(273.1));
        public static Translation2d StartingPointB = new Translation2d(Units.inchesToMeters(25.6), Units.inchesToMeters(265.6)); //60 degrees
        public static Rotation2d RotationOfStartingPointB = Rotation2d.fromDegrees(60);
        public static Translation2d StartingPointC = new Translation2d(Units.inchesToMeters(53.1 + 3.125), Units.inchesToMeters(219.6));
        public static Translation2d StartingPointD = new Translation2d(Units.inchesToMeters(25.6), Units.inchesToMeters(170.6));
        public static Rotation2d RotationOfStartingPointD = Rotation2d.fromDegrees(-60);
        public static Translation2d StartingPointE = new Translation2d(Units.inchesToMeters(18.6), Units.inchesToMeters(138.6));
        public static Translation2d StartingPointF = new Translation2d(Units.inchesToMeters(18.6), Units.inchesToMeters(92.6));

        public static Translation2d StageEnterTop = new Translation2d(Units.inchesToMeters(158.6),Units.inchesToMeters(216));
        public static Rotation2d RotationOfStageEnterTop = Rotation2d.fromDegrees(-120);
        public static Translation2d StageEnterBottom = new Translation2d(Units.inchesToMeters(158.6),Units.inchesToMeters(106.6));
        public static Rotation2d RotationOfStageBottomTop = Rotation2d.fromDegrees(60);

        public static Translation2d CenterStage = new Translation2d(Units.inchesToMeters(193.6),Units.inchesToMeters(161.6));
        public static Translation2d OutOfStage = new Translation2d(Units.inchesToMeters(258.6),Units.inchesToMeters(161.6));

        public static Translation2d DetourPointBottom = new Translation2d(Units.inchesToMeters(233.6), Units.inchesToMeters(44.6));
        public static Translation2d DetourPointTop = new Translation2d(Units.inchesToMeters(233.6), Units.inchesToMeters(270.6));

    }
}
