// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.generated.CompTunerConstants;
import frc.robot.util.LoggedTracer;


/** Represents a mecanum drive style drivetrain. */
public class Drive extends SubsystemBase{
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  public final double BaseRadius;

  //private final GyroIO gyro;
  private final GyroIOInputsAutoLogged gyroInputs= new GyroIOInputsAutoLogged();
  private final MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;
  private Rotation2d gyroRotation= Rotation2d.kZero;
  static final Lock odometryLock = new ReentrantLock();
  public static int debug=1;

  private final Mecanum mecanum[] = new Mecanum[4];

  /** Constructs a MecanumDrive and resets the gyro. */
  public Drive(/*GyroIO gyro*/ ) {
    //this.gyro= gyro;

    if(Constants.getRobot()== RobotType.COMPETITION){
      mecanum[0]= new Mecanum(new MecanumHardwareIO(1, false), "FrontLeft");
      mecanum[1]= new Mecanum(new MecanumHardwareIO(2, true), "FrontRight");
      mecanum[2]= new Mecanum(new MecanumHardwareIO(3, false), "BackLeft");
      mecanum[3]= new Mecanum(new MecanumHardwareIO(4, true), "BackRight");
    } else if(Constants.getRobot()==RobotType.SIMBOT){
      mecanum[0]= new Mecanum(new MecanumSimIO(1, false), "FrontLeft");
      mecanum[1]= new Mecanum(new MecanumSimIO(2, true), "FrontRight");
      mecanum[2]= new Mecanum(new MecanumSimIO(3, false), "BackLeft");
      mecanum[3]= new Mecanum(new MecanumSimIO(4, true), "BackRight");
    }

    Translation2d[] wheelLocations= getWheelLocations();
    kinematics= new MecanumDriveKinematics(wheelLocations[0], wheelLocations[1],
    wheelLocations[2], wheelLocations[3]);

    odometry= new MecanumDriveOdometry(kinematics, gyroRotation, 
    new MecanumDriveWheelPositions(mecanum[0].getWheelPositions(), mecanum[1].getWheelPositions(),
    mecanum[2].getWheelPositions(), mecanum[3].getWheelPositions()), 
    Pose2d.kZero);
    debug=1;

    BaseRadius= 
      Math.max(
            Math.max(
            Math.hypot(MecanumConstants.FrontLeft.getX(), MecanumConstants.FrontLeft.getY()),
            Math.hypot(MecanumConstants.FrontRight.getX(), MecanumConstants.FrontRight.getY())),
            Math.max(
            Math.hypot(MecanumConstants.BackLeft.getX(), MecanumConstants.BackLeft.getY()),
            Math.hypot(MecanumConstants.BackRight.getX(), MecanumConstants.BackRight.getY())));

  }
  
  @Override
  public void periodic(){
    LoggedTracer.reset();
    //gyro.updateInputs(gyroInputs);
    Logger.processInputs("Gyro", gyroInputs);

    Logger.recordOutput("Wheel/Setpoint", RPM.of(500));

    /*for(var i=0; i<4; i++){
      mecanum[i].periodic();
    }*/

    /*  if(DriverStation.isDisabled()){
      for(var i=0; i<4; i++){
        mecanum[i].stop();
      }
    }
    */
    Logger.recordOutput("DebugDev", debug);

    //Getting new wheel positions
    var lastWheels= new MecanumDriveWheelPositions(mecanum[0].getWheelPositions(), mecanum[1].getWheelPositions(),
    mecanum[2].getWheelPositions(), mecanum[3].getWheelPositions());
    //Getting new gyro rotation
    gyroRotation= gyroInputs.yawPosition;
    //Updating robot pose
    var robotPose= odometry.update(gyroRotation, lastWheels);
  }

  public void stop(){
    for(var i=0; i<4; i++){
      mecanum[i].stop();
    }
    debug=5;
    Logger.recordOutput("DebugDev", debug);
  }

  public void runVelocity(ChassisSpeeds speeds){
    ChassisSpeeds discreteSpeeds= ChassisSpeeds.discretize(speeds, 0.02);
    MecanumDriveWheelSpeeds wheelSpeeds= kinematics.toWheelSpeeds(discreteSpeeds);

    wheelSpeeds.desaturate(MecanumConstants.maxLinearSpeedMetersPerSecond);
    //debug=200;
    mecanum[0].setSetpoint(wheelSpeeds.frontLeftMetersPerSecond);
    mecanum[1].setSetpoint(wheelSpeeds.frontRightMetersPerSecond);
    mecanum[2].setSetpoint(wheelSpeeds.rearLeftMetersPerSecond);
    mecanum[3].setSetpoint(wheelSpeeds.rearRightMetersPerSecond);
    
    Logger.recordOutput("DebugDev", debug);
  }

  /*public void runVelocity(LinearVelocity x, LinearVelocity y, AngularVelocity omega){
    runVelocity(new ChassisSpeeds(x, y, omega));
  }*/

  public Command runVelocityCmd(ChassisSpeeds speeds){
    return startEnd(()-> {runVelocity(speeds);}, this::stop);
  }

  public Command runLinearCmd(LinearVelocity x, LinearVelocity y, AngularVelocity omega){

    return runVelocityCmd(new ChassisSpeeds(x, y, omega));
  }

  public double getMaxLinearSpeed(){
    return MecanumConstants.maxLinearSpeedMetersPerSecond;
  }

  public double getMaxAngularSpeed(){
    return (MecanumConstants.maxLinearSpeedMetersPerSecond)/(BaseRadius);
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