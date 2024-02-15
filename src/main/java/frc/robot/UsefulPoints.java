// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class UsefulPoints {
    public static class Points {
        public static Translation2d middleOfSpeaker = new Translation2d(
                0,Units.inchesToMeters(219));
        public static Translation2d WingedNoteLeft = new Translation2d(Units.inchesToMeters(27*12 - 211.6), Units.inchesToMeters(27*6 + 114));
        public static Translation2d WingedNoteMid = new Translation2d(Units.inchesToMeters(27*12 - 211.6), Units.inchesToMeters(27*6 + 57));
        public static Translation2d WingedNoteRight = new Translation2d(Units.inchesToMeters(27*12 - 211.6),Units.inchesToMeters(27*6));

        public static Translation2d CenterLineMid = new Translation2d(Units.inchesToMeters(27*12), Units.inchesToMeters(27*6));
        public static Translation2d CenterLineFarLeft = new Translation2d(Units.inchesToMeters(27*12), Units.inchesToMeters(27*6 + 132));
        public static Translation2d CenterLineMidLeft = new Translation2d(Units.inchesToMeters(27*12), Units.inchesToMeters(27*6 + 66));
        public static Translation2d CenterLineFarRight = new Translation2d(Units.inchesToMeters(27*12), Units.inchesToMeters(27*6 - 132));
        public static Translation2d CenterLineMidRight = new Translation2d(Units.inchesToMeters(27*12), Units.inchesToMeters(27*6 -66));

        public static Translation2d StartingPointA = new Translation2d(Units.inchesToMeters(26), Units.inchesToMeters(27*6 + 124));
        public static Translation2d StartingPointB = new Translation2d(Units.inchesToMeters(26), Units.inchesToMeters(27*6 + 72));
        public static Translation2d StartingPointC = new Translation2d(Units.inchesToMeters(26), Units.inchesToMeters(27*6 + 0));



    }
}
