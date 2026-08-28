package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;


import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;


import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class DriveCommands {
  private static final double DEADBAND = 0.1;

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

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities). This is preconfigured with {@link #configure(Drive, DoubleSupplier, DoubleSupplier, DoubleSupplier)}
   */
  public static Command joystickDriveField() {
    if (!configured) throw new IllegalStateException("DriveCommands joystickDriveField called without first configuring!");
    
    return joystickDriveField(drive_, xSupplier_, ySupplier_, omegaSupplier_);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   * @param drive
   * @param xSupplier
   * @param ySupplier
   * @param omegaSupplier
   * @return
   */
  public static Command joystickDriveField(
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
          //omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeed(),
              linearVelocity.getY() * drive.getMaxLinearSpeed(),
              omega * drive.getMaxAngularSpeed());
              Logger.recordOutput("Omega/Supplier", omega);

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds, drive.getRotation().unaryMinus())
          );
        },
        drive::stop
    );
  }

  public static Command joystickDriveRobot() {
    if (!configured) throw new IllegalStateException("DriveCommands joystickDriveRobot called without first configuring!");
    
    return joystickDriveRobot(drive_, xSupplier_, ySupplier_, omegaSupplier_);
  }
  /**
   * Robot relative drive command using two joysticks (controlling linear and
   * angular velocities).
   * @param drive
   * @param xSupplier
   * @param ySupplier
   * @param omegaSupplier
   * @return
   */
  public static Command joystickDriveRobot(
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
          //omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds = new ChassisSpeeds(
              linearVelocity.getX() * drive.getMaxLinearSpeed(),
              linearVelocity.getY() * drive.getMaxLinearSpeed(),
              omega * drive.getMaxAngularSpeed());
              Logger.recordOutput("Omega/Supplier", omega);
          
          drive.runVelocity(
            speeds
          );
        },
        drive::stop
    );
  }
  
  
  
  public static Command followPathCommand(String pathName, Drive drive) {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        var startingPose= path.getStartingHolonomicPose().orElseThrow();
        return Commands.sequence(
          Commands.runOnce(()-> drive.resetPose(startingPose)),
          AutoBuilder.followPath(path)
        );
        
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  public static Command slowDriveField(
    Drive drive,
    DoubleSupplier xSupplier,
    DoubleSupplier ySupplier,
    DoubleSupplier omegaSupplier
  ){
    return drive.runEnd(()->{
      ChassisSpeeds speed= new ChassisSpeeds(
        xSupplier.getAsDouble() * drive.getMaxLinearSpeed(),
        ySupplier.getAsDouble() * drive.getMaxLinearSpeed(),
        omegaSupplier.getAsDouble() * drive.getMaxAngularSpeed()
      );

      drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(speed, 
        drive.getRotation().unaryMinus())
      );
    }, drive::stop);
  }
}