package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.test.SparkMax;

public class AutoCommands {
    public static Command test(Drive drive){
        return Commands.sequence(
            drive.zeroGyro(),
            DriveCommands.followPathCommand("Test", drive)
        );
    }

    public static Command straight(Drive drive, SparkMax testSpark){
        return Commands.sequence(
            DriveCommands.joystickDrive(drive, ()->0.8, ()->0.0, ()->0.0).deadlineFor(testSpark.bypass(0.8)).withTimeout(3)
        );
    }
}
