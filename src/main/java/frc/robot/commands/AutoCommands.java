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

    public static Command redSideClose(Drive drive, SparkMax testSpark){
        return Commands.sequence(
            drive.zeroGyro(),
            DriveCommands.joystickDriveRobot(drive, ()->0.8, ()->-0.15, ()->0.0).deadlineFor(testSpark.bypass(1.0)).withTimeout(1.0),
            DriveCommands.joystickDriveRobot(drive, ()-> 0.2, ()->0.0, ()-> 0.6).deadlineFor(testSpark.bypass(1.0)).withTimeout(0.35),
            DriveCommands.joystickDriveRobot(drive, ()->0.9, ()->0.0, ()->0.0).deadlineFor(testSpark.bypass(1.0)).withTimeout(0.65),
            DriveCommands.joystickDriveRobot(drive, ()->0.0, ()-> 0.0, ()-> 0.7).withTimeout(0.5),
            DriveCommands.joystickDriveRobot(drive, ()-> 0.8, ()-> 0.12, ()-> 0.0).withTimeout(0.8),
            DriveCommands.joystickDriveRobot(drive, ()->0.0, ()-> 0.0, ()->-0.7).withTimeout(0.4),
            DriveCommands.joystickDriveRobot(drive, ()->0.7, ()-> 0.0, ()-> 0.0).deadlineFor(testSpark.bypass(-1.0)).withTimeout(0.6),
            testSpark.bypass(-1.0).withTimeout(1.0),
            DriveCommands.joystickDriveRobot(drive, ()->-0.8, ()-> 0.0, ()-> 0.0).withTimeout(1)
        );
    }
    public static Command redSide(Drive drive, SparkMax testSpark){
        return Commands.sequence(
            drive.zeroGyro(),
            DriveCommands.joystickDriveRobot(drive, ()->0.8, ()->0.0, ()->0.0).deadlineFor(testSpark.bypass(1.0)).withTimeout(1.0),
            DriveCommands.joystickDriveRobot(drive, ()-> 0.2, ()->0.0, ()-> 0.6).deadlineFor(testSpark.bypass(1.0)).withTimeout(0.35),
            DriveCommands.joystickDriveRobot(drive, ()->0.9, ()->0.0, ()->0.0).deadlineFor(testSpark.bypass(1.0)).withTimeout(0.75),
            DriveCommands.joystickDriveRobot(drive, ()->0.0, ()-> 0.0, ()-> 0.7).withTimeout(0.5),
            DriveCommands.joystickDriveRobot(drive, ()-> 0.8, ()-> 0.2, ()-> 0.0).withTimeout(0.8),
            DriveCommands.joystickDriveRobot(drive, ()->0.0, ()-> 0.0, ()->-0.7).withTimeout(1),
            DriveCommands.joystickDriveRobot(drive, ()->0.6, ()-> 0.0, ()-> 0.0).deadlineFor(testSpark.bypass(-1.0)).withTimeout(0.6),
            DriveCommands.joystickDriveRobot(drive, ()->-0.8, ()-> 0.0, ()-> 0.0).withTimeout(1)
        );
    }

    public static Command blueSide(Drive drive, SparkMax testSpark){
        return Commands.sequence(
            drive.zeroGyro(),
            DriveCommands.joystickDriveRobot(drive, ()->0.8, ()->-0.15, ()->0.0).deadlineFor(testSpark.bypass(1.0)).withTimeout(1.0),
            DriveCommands.joystickDriveRobot(drive, ()-> 0.2, ()->0.0, ()-> 0.6).deadlineFor(testSpark.bypass(1.0)).withTimeout(0.35),
            DriveCommands.joystickDriveRobot(drive, ()->0.9, ()->0.0, ()->0.0).deadlineFor(testSpark.bypass(1.0)).withTimeout(0.65),
            DriveCommands.joystickDriveRobot(drive, ()->0.0, ()-> 0.0, ()-> 0.7).withTimeout(0.5),
            DriveCommands.joystickDriveRobot(drive, ()-> 0.8, ()-> 0.12, ()-> 0.0).withTimeout(1.05),
            DriveCommands.joystickDriveRobot(drive, ()->0.0, ()-> 0.0, ()->-0.7).withTimeout(0.4),
            DriveCommands.joystickDriveRobot(drive, ()->0.6, ()-> 0.0, ()-> 0.0).deadlineFor(testSpark.bypass(-1.0)).withTimeout(0.75),
            //testSpark.bypass(-1.0).withTimeout(1.0),
            DriveCommands.joystickDriveRobot(drive, ()->-0.8, ()-> 0.0, ()-> 0.0).withTimeout(1)
        );
    }
}
