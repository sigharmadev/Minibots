package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class MecanumConstants {
    public static final int ENCODER_COUNTS_PER_REVOLUTION = 538;
    public static final boolean ENCODER_INVERTED = false;

    //Change these for later when tuning
    public static final double kP= 0.006; 
    public static final double kI= 0.0;
    public static final double kD= 0.0;
    public static final double kS= 0.0;
    public static final double kV= 0.0;
    public static final double kA= 0.0;

    public static final double cruiseVelocity= 75;
    public static final double acceleration= 100; //RPM/s
    public static final double allowedProfileError= 5.0; //RPM

    public static final double MOI= 0.01;
    public static final double DriveGearRatio= 1.0; //might need to change
    public static final double wheelRadius=0.048; //definitely need to change

    //Change these later
    public static final Translation2d FrontLeft= new Translation2d(-0.21, 0.17);
    public static final Translation2d FrontRight= new Translation2d(0.21, 0.17);
    public static final Translation2d BackLeft= new Translation2d(-0.21, -0.17);
    public static final Translation2d BackRight= new Translation2d(0.21, -0.17);  

    public static final double maxLinearSpeedMetersPerSecond= 1.0;
}