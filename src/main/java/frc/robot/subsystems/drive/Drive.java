

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs;
import frc.robot.util.LoggedTracer;


/** Represents a mecanum drive style drivetrain. */
public class Drive extends SubsystemBase{
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  public final double BaseRadius;

  private final GyroIO gyro;
  private final MecanumDriveKinematics kinematics;
  private MecanumDriveOdometry odometry;
  public static int debug=1;
  private final Mecanum mecanum[] = new Mecanum[4];
  private final GyroIOInputsAutoLogged gyroInputs= new GyroIOInputsAutoLogged();
  private Rotation2d gyroRotation= Rotation2d.kZero;

  public Drive(GyroIO gyro) {
    Translation2d[] wheelLocations= getWheelLocations();
    this.kinematics= new MecanumDriveKinematics(wheelLocations[0], wheelLocations[1],
    wheelLocations[2], wheelLocations[3]);

    if (Constants.getRobot() == RobotType.COMPETITION) {
      this.gyro = gyro; 

      mecanum[0] = new Mecanum(new MecanumHardwareIO(1, false), "FrontRight");
      mecanum[1] = new Mecanum(new MecanumHardwareIO(2, true), "FrontLeft");
      mecanum[2] = new Mecanum(new MecanumHardwareIO(3, true), "BackLeft");
      mecanum[3] = new Mecanum(new MecanumHardwareIO(4, false), "BackRight");
  
    } else {
      mecanum[0] = new Mecanum(new MecanumSimIO(1, false), "FrontRight");
      mecanum[1] = new Mecanum(new MecanumSimIO(2, true), "FrontLeft");
      mecanum[2] = new Mecanum(new MecanumSimIO(3, true), "BackLeft");
      mecanum[3] = new Mecanum(new MecanumSimIO(4, false), "BackRight");

      // First and only assignment for simulation
      this.gyro = new GyroIOSim(kinematics, mecanum); 
    }
    
    odometry= new MecanumDriveOdometry(kinematics, gyroRotation, 
    new MecanumDriveWheelPositions(mecanum[1].getWheelPositions(), mecanum[0].getWheelPositions(),
    mecanum[2].getWheelPositions(), mecanum[3].getWheelPositions()), 
    Pose2d.kZero);

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
    gyro.updateInputs(gyroInputs);
    Logger.processInputs("Gyro",gyroInputs);


    //Getting new wheel positions
    var lastWheels= new MecanumDriveWheelPositions(mecanum[1].getWheelPositions(), mecanum[0].getWheelPositions(),
    mecanum[2].getWheelPositions(), mecanum[3].getWheelPositions());
    //Getting new gyro rotation
    gyroRotation= gyroInputs.yawPosition;
    //Updating robot pose
    var robotPose= odometry.update(gyroRotation, lastWheels);
    Logger.recordOutput("RobotPose/X", robotPose.getX());
    Logger.recordOutput("RobotPose/Y", robotPose.getY());
    Logger.recordOutput("RobotPose/Rotation", robotPose.getRotation().getRadians());
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

    wheelSpeeds.desaturate(MecanumConstants.maxWheelSpeedMetersPerSecond);
    //debug=200;
    mecanum[0].setSetpoint(wheelSpeeds.frontRightMetersPerSecond);
    mecanum[1].setSetpoint(wheelSpeeds.frontLeftMetersPerSecond);
    mecanum[2].setSetpoint(wheelSpeeds.rearLeftMetersPerSecond);
    mecanum[3].setSetpoint(wheelSpeeds.rearRightMetersPerSecond);
    
    Logger.recordOutput("DebugDev", debug);
  }


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

  public void bypassDuty(ChassisSpeeds speeds){
    ChassisSpeeds discreteSpeeds= ChassisSpeeds.discretize(speeds, 0.02);
    MecanumDriveWheelSpeeds wheelSpeeds= kinematics.toWheelSpeeds(discreteSpeeds);

    Logger.recordOutput("Omega/Discrete", discreteSpeeds.omegaRadiansPerSecond);

    wheelSpeeds.desaturate(MecanumConstants.maxWheelSpeedMetersPerSecond);

    Logger.recordOutput("WheelSpeeds/FrontRight", wheelSpeeds.frontRightMetersPerSecond);
    Logger.recordOutput("WheelSpeeds/FrontLeft", wheelSpeeds.frontLeftMetersPerSecond);
    Logger.recordOutput("WheelSpeeds/BackLeft", wheelSpeeds.rearLeftMetersPerSecond);
    Logger.recordOutput("WheelSpeeds/BackRight", wheelSpeeds.rearRightMetersPerSecond);
    Logger.recordOutput("WheelSpeeds/MaxWheelMetersPerSecond", MecanumConstants.maxWheelSpeedMetersPerSecond);

    double frontRightAngular= 60.0*(wheelSpeeds.frontRightMetersPerSecond/(MecanumConstants.wheelRadius))/(2.0*Math.PI);
    double frontLeftAngular= 60.0*(wheelSpeeds.frontLeftMetersPerSecond/(MecanumConstants.wheelRadius))/(2.0*Math.PI);
    double backLeftAngular= 60.0*(wheelSpeeds.rearLeftMetersPerSecond/(MecanumConstants.wheelRadius))/(2.0*Math.PI);
    double backRightAngular= 60.0*(wheelSpeeds.rearRightMetersPerSecond/(MecanumConstants.wheelRadius))/(2.0*Math.PI);

    Logger.recordOutput("WheelSpeeds/FrontRightAngular", frontRightAngular);
    Logger.recordOutput("WheelSpeeds/FrontLeftAngular", frontLeftAngular);
    Logger.recordOutput("WheelSpeeds/BackRightAngular", backRightAngular);
    Logger.recordOutput("WheelSpeeds/BackLeftAngular", backLeftAngular);


    mecanum[0].duty((frontRightAngular/312)*1.0);
    mecanum[1].duty((frontLeftAngular/312)*1.0);
    mecanum[2].duty((backLeftAngular/312)*1.0);
    mecanum[3].duty((backRightAngular/312)*1.0);

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