package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;

public class AutoCommands {
    public static Command test(Drive drive){
        return Commands.sequence(
            drive.zeroGyro(),
            DriveCommands.followPathCommand("Test", drive)
        );
    }
}
