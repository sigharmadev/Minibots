package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;

public class GyroIOSim extends NavXIO{
  private Rotation2d yaw = Rotation2d.kZero;
  private final MecanumDriveKinematics kinematics;
  private final Mecanum[] mecanums;

  public GyroIOSim(MecanumDriveKinematics kinematics, Mecanum[] mecanums) {
    this.kinematics = kinematics;
    this.mecanums = mecanums;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // 1. Grab simulated wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
      mecanums[1].getWheelVelocity(), // FrontLeft
      mecanums[0].getWheelVelocity(), // FrontRight
      mecanums[2].getWheelVelocity(), // BackLeft
      mecanums[3].getWheelVelocity()  // BackRight
    );

    // 2. Convert wheel speeds back to chassis movement math
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);

    // 4. Update inputs for AdvantageKit
    inputs.yawPosition = yaw;
    inputs.yawVelocityRadPerSec = chassisSpeeds.omegaRadiansPerSecond;
  }
}