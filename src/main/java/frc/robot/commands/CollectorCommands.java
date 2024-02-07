// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.HeadSubsystem;

import java.util.function.BiFunction;

public final class CollectorCommands {
    /** Example static factory for an autonomous command. */


    // commands to set shoulder and wrist at various positions


    private static Command moveThenCollect(
            HeadSubsystem headSubsystem,
            Arm armSubsystem,
            Rotation2d shoulder,
            Rotation2d wrist,
            double topSpeed,
            double bottomSpeed
    ){

        var command = Commands.parallel(

                /*armSubsystem.followPathCommand(
                        (shoulder),
                        (wrist)
                ),*/

                headSubsystem.runCollectorCommands(topSpeed,bottomSpeed)

        );

        return command;


    }

    // floor pickup
    public static Command headDownThenCollect(HeadSubsystem headSubsystem, Arm armSubsystem) {

        return moveThenCollect(headSubsystem,armSubsystem,  Rotation2d.fromDegrees(90),
                Rotation2d.fromDegrees(45), 0.5, 0.5 );
    }


    // Amp pickup
    public static Command setArmToAmpThenDepositCollector(HeadSubsystem headSubsystem, Arm armSubsystem){

        return moveThenCollect(headSubsystem,armSubsystem,  Rotation2d.fromDegrees(135),
                Rotation2d.fromDegrees(135), -0.5, -0.5 );

    }

    // TODO write new command for Shooter for AMP(not done)
    // TODO write command so that it will wait until is collected(not done)
    // TODO Make one big command that takes in position and speeds to a void copy pasting(done)
    // Source Collect
    public static Command setArmToSourceThenCollect(HeadSubsystem headSubsystem, Arm armSubsystem){

        return moveThenCollect(headSubsystem,armSubsystem,  Rotation2d.fromDegrees(155),
                Rotation2d.fromDegrees(135), 0.5, 0.5 );

    }





    // angles need to be changed to fit arm

    public static Command setArmToAmpThenDepositShooter(HeadSubsystem headSubsystem, Arm armSubsystem){

        return moveThenCollect(headSubsystem,armSubsystem,  Rotation2d.fromDegrees(-155),
                Rotation2d.fromDegrees(-135), 0.5, 0.5 );

    }




}
