// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.headSubsystem;

public final class CollectorCommands {
    /** Example static factory for an autonomous command. */
    public static Command headDownThenCollect(headSubsystem headSubsystem, Arm armSubsystem) {
        var headDownThenCollect = Commands.sequence(
                armSubsystem.followPathCommand(
                        Rotation2d.fromDegrees(90),
                        Rotation2d.fromDegrees(45)
                ),
                new CollectorOnOrOffCommand(headSubsystem, true),
                new setCollectorCommand(headSubsystem, 3000)
        );

        return headDownThenCollect;
    }

    public static Command setArmToAmpThenDeposit(headSubsystem headSubsystem, Arm armSubsystem){
        var setArmToAmpThenDeposit = Commands.sequence(
                armSubsystem.followPathCommand(
                        Rotation2d.fromDegrees(135),
                        Rotation2d.fromDegrees(135)
                ),
                new CollectorOnOrOffCommand(headSubsystem, true),
                new setCollectorCommand(headSubsystem, -3000)
        );

        return setArmToAmpThenDeposit;
    }

    public static Command setArmToSourceThenCollect(headSubsystem headSubsystem, Arm armSubsystem){
        var setArmToSourceThenCollect = Commands.sequence(
                armSubsystem.followPathCommand(
                        Rotation2d.fromDegrees(155),
                        Rotation2d.fromDegrees(135)
                ),
                new CollectorOnOrOffCommand(headSubsystem, true),
                new setCollectorCommand(headSubsystem, 3000)
        );

        return setArmToSourceThenCollect;
    }




}
