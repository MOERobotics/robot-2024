// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class UsefulPoints {
    public static class Points {
        public Translation2d middleOfSpeaker = new Translation2d(
                0,Units.inchesToMeters(219));
        public Translation2d WingedNoteLeft = new Translation2d(Units.inchesToMeters(27*12 - 211.6), Units.inchesToMeters(27*6 + 114));
        public Translation2d WingedNoteMid = new Translation2d(Units.inchesToMeters(27*12 - 211.6), Units.inchesToMeters(27*6 + 57));
        public Translation2d WingedNoteRight = new Translation2d(Units.inchesToMeters(27*12 - 211.6),Units.inchesToMeters(27*6));

        public Translation2d CenterLineMid = new Translation2d(Units.inchesToMeters(27*12), Units.inchesToMeters(27*6));
        public Translation2d CenterLineFarLeft = new Translation2d(Units.inchesToMeters(27*12), Units.inchesToMeters(27*6 + 132));
        public Translation2d CenterLineMidLeft = new Translation2d(Units.inchesToMeters(27*12), Units.inchesToMeters(27*6 + 66));
        public Translation2d CenterLineFarRight = new Translation2d(Units.inchesToMeters(27*12), Units.inchesToMeters(27*6 - 132));
        public Translation2d CenterLineMidRight = new Translation2d(Units.inchesToMeters(27*12), Units.inchesToMeters(27*6 -66));




    }
}
