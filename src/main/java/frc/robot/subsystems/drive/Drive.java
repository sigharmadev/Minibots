// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTracer;

/** Represents a mecanum drive style drivetrain. */
public class Drive extends SubsystemBase{
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // These Constants should be the same for every drivebase, so just use the comp bot constants.
  static final double ODOMETRY_FREQUENCY = CompTunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public final double DRIVE_BASE_RADIUS;

  // Gyro degrees-per-rotation correction/trim
  static final double GYRO_YAW_DEG_PER_ROT_CORRECTION = -0.97;

  // These constants should change for every drivebase
  private final LinearVelocity SPEED_12_VOLTS;
  private final RobotConfig PP_CONFIG;
    
  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 65.7709;
  private static final double ROBOT_MOI = 6.33;
  private static final double WHEEL_COF = 1.2;

  private final GyroIO gyro;
  private final GyroIOInputsAutoLogged gyroInputs= new GyroIOInputsAutoLogged();
  private final MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;
  private Rotation2d gyroRotation= Rotation2d.kZero;
  static final Lock odometryLock = new ReentrantLock();

  private final Mecanum mecanum[] = new Mecanum[4];

  /** Constructs a MecanumDrive and resets the gyro. */
  public Drive(GyroIO gyro) {
    this.gyro= gyro;
    mecanum[0]= new Mecanum(new MecanumHardwareIO(0));
    mecanum[1]= new Mecanum(new MecanumHardwareIO(1));
    mecanum[2]= new Mecanum(new MecanumHardwareIO(2));
    mecanum[3]= new Mecanum(new MecanumHardwareIO(3));

    Translation2d[] wheelLocations= getWheelLocations();
    kinematics= new MecanumDriveKinematics(wheelLocations[0], wheelLocations[1],
    wheelLocations[2], wheelLocations[3]);

    odometry= new MecanumDriveOdometry(kinematics, gyroRotation, 
    new MecanumDriveWheelPositions(mecanum[0].getWheelPositions(), mecanum[1].getWheelPositions(),
    mecanum[2].getWheelPositions(), mecanum[3].getWheelPositions()), 
    Pose2d.kZero);
  }
  
  @Override
  public void periodic(){
    LoggedTracer.reset();
    gyro.updateInputs(gyroInputs);
    Logger.process(gyroInputs);

    for(var i=0; i<4; i++){
      mecanum[i].periodic();
    }

    if(DriverStation.isDisabled()){
      for(var i=0; i<4; i++){
        mecanum[i].stop();
      }
    }

    //Getting new wheel positions
    var lastWheels= new MecanumDriveWheelPositions(mecanum[0].getWheelPositions(), mecanum[1].getWheelPositions(),
    mecanum[2].getWheelPositions(), mecanum[3].getWheelPositions());
    //Getting new gyro rotation
    gyroRotation= gyroInputs.yawPosition;
    //Updating robot pose
    var robotPose= odometry.update(gyroRotation, lastWheels);
  }

  public void runVelocity(ChassisSpeeds speeds){
    ChassisSpeeds discreteSpeeds= ChassisSpeeds.discretize(speeds, 0.02);
    MecanumDriveWheelSpeeds wheelSpeeds= kinematics.toWheelSpeeds(discreteSpeeds);

    for(var i=0; i<4; i++){
      mecanum[i].runVelocity(wheelSpeeds);
    }
  }

  
  public Translation2d[] getWheelLocations(){
    return new Translation2d[]{
      MecanumConstants.FrontLeft,
      MecanumConstants.FrontRight,
      MecanumConstants.BackLeft,
      MecanumConstants.BackRight
    };
  }
}