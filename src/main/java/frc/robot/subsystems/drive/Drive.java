

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
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
  private MecanumDrivePoseEstimator poseEstimator;
  private Rotation2d gyroOffset = Rotation2d.kZero;
  public RobotConfig config;
  private MecanumDriveWheelPositions lastWheels= new MecanumDriveWheelPositions(
    0,0,0,0
  );


  public Drive(GyroIO gyro) {
    Translation2d[] wheelLocations= getWheelLocations();
    this.kinematics= new MecanumDriveKinematics(wheelLocations[0], wheelLocations[1],
    wheelLocations[2], wheelLocations[3]);
    this.gyro = gyro; 
    if (Constants.getRobot() == RobotType.COMPETITION) {
      mecanum[0] = new Mecanum(new MecanumHardwareIO(1, false), "FrontRight");
      mecanum[1] = new Mecanum(new MecanumHardwareIO(2, true), "FrontLeft");
      mecanum[2] = new Mecanum(new MecanumHardwareIO(3, true), "BackLeft");
      mecanum[3] = new Mecanum(new MecanumHardwareIO(4, false), "BackRight");
  
    } else {
      mecanum[0] = new Mecanum(new MecanumSimIO(1, false), "FrontRight");
      mecanum[1] = new Mecanum(new MecanumSimIO(2, true), "FrontLeft");
      mecanum[2] = new Mecanum(new MecanumSimIO(3, true), "BackLeft");
      mecanum[3] = new Mecanum(new MecanumSimIO(4, false), "BackRight");
    }
    
    poseEstimator= new MecanumDrivePoseEstimator(kinematics, gyroRotation, 
      lastWheels,
      Pose2d.kZero);

    config= new RobotConfig(
      Config.MASS_KG_ROBOT,
      Config.MOI_ROBOT,
      new ModuleConfig(
        MecanumConstants.wheelRadius, 
        MecanumConstants.maxLinearSpeedMetersPerSecond,
        1.0,
        new DCMotor(12, 24.3, 9.2, 0.25, (104*Math.PI), 1),
        9.2, 
        1),
      getWheelLocations()
    );

    BaseRadius= 
      Math.max(
            Math.max(
            Math.hypot(MecanumConstants.FrontLeft.getX(), MecanumConstants.FrontLeft.getY()),
            Math.hypot(MecanumConstants.FrontRight.getX(), MecanumConstants.FrontRight.getY())),
            Math.max(
            Math.hypot(MecanumConstants.BackLeft.getX(), MecanumConstants.BackLeft.getY()),
            Math.hypot(MecanumConstants.BackRight.getX(), MecanumConstants.BackRight.getY())));

    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards)-> bypassDuty(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

  }
  
  @Override
  public void periodic(){
    LoggedTracer.reset();
    gyro.updateInputs(gyroInputs);
    Logger.processInputs("Gyro",gyroInputs);
    //Getting new wheel positions
    lastWheels= new MecanumDriveWheelPositions(mecanum[1].getWheelPositions(), mecanum[0].getWheelPositions(),
    mecanum[2].getWheelPositions(), mecanum[3].getWheelPositions());
    //Getting new gyro rotation
    gyroRotation= getRotation();
    //Updating robot pose
    poseEstimator.update(gyroRotation, lastWheels);
    Logger.recordOutput("Pose/x", poseEstimator.getEstimatedPosition().getX());
    Logger.recordOutput("Pose/y", poseEstimator.getEstimatedPosition().getY());
    Logger.recordOutput("Pose/theta", poseEstimator.getEstimatedPosition().getRotation().getRadians());
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

  public Rotation2d getRotation() {
    // Subtract the offset so that whatever position the robot was in 
    // when you pressed reset becomes the new "0"
    return gyroInputs.yawPosition.minus(gyroOffset);
  }

  public Command zeroGyro() {
    // Store the current raw position as our new zero-point
    return Commands.runOnce(()->this.gyroOffset = gyroInputs.yawPosition);
  }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose){
    poseEstimator.resetPosition(getRotation(), lastWheels, pose);
  }

  public ChassisSpeeds getChassisSpeeds(){
    return kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(
      mecanum[1].getWheelVelocity(),
      mecanum[0].getWheelVelocity(),
      mecanum[2].getWheelVelocity(),
      mecanum[3].getWheelVelocity()
    ));
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