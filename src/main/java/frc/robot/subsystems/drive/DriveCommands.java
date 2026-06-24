// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.MapleSimUtil;

public class DriveCommands {
  private static final double kStoppedVelocity = 0.15 ;

  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 4.0;
  private static final double ANGLE_KD = 0.0;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 40.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private static Drive drive_;
  private static DoubleSupplier xSupplier_;
  private static DoubleSupplier ySupplier_;
  private static DoubleSupplier omegaSupplier_;
  private static boolean configured = false;

  private DriveCommands() {}

  /**
   * Configures the drive commands. In order to call convenience drive commands, this must be configured beforehand.
   * @param drive Drive subsystem
   * @param xSupplier Supplier of X velocity (negative left joystick Y)
   * @param ySupplier Supplier of Y velocity (negative left joystick X)
   * @param omegaSupplier Supplier of rotational velocity (negative right joystick X)
   */
  public static void configure(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    drive_ = drive;
    xSupplier_ = xSupplier;
    ySupplier_ = ySupplier;
    omegaSupplier_ = omegaSupplier;

    configured = true;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static Pose2d rotateIfRed(Pose2d pose) {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      Translation2d center = new Translation2d(FieldConstants.layout.getFieldLength() / 2.0, FieldConstants.layout.getFieldWidth() / 2.0);
      pose = pose.rotateAround(center, Rotation2d.fromDegrees(180.0)) ;
    }

    return pose ;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities). This is preconfigured with {@link #configure(Drive, DoubleSupplier, DoubleSupplier, DoubleSupplier)}
   */
  public static Command joystickDrive() {
    if (!configured) throw new IllegalStateException("DriveCommands joystickDrive called without first configuring!");
    
    return joystickDrive(drive_, xSupplier_, ySupplier_, omegaSupplier_);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return drive.runEnd(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeed(),
              linearVelocity.getY() * drive.getMaxLinearSpeed(),
              omega * drive.getMaxAngularSpeed());
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
            speeds
          );
        },
        drive::stop
    );
  }
  
}