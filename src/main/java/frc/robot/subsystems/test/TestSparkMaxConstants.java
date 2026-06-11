package frc.robot.subsystems.test;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class TestSparkMaxConstants {
    public static final int CIM_MOTOR_ID = 2;
    public static final int ENCODER_COUNTS_PER_REVOLUTION = 8192;
    public static final boolean ENCODER_INVERTED = true;

    public static final double kP= 0.0;
    public static final double kI= 0.0;
    public static final double kD= 0.0;
    public static final double kS= 0.0;
    public static final double kV= 0.0;
    public static final double kA= 0.0;

    public static final double velocitySetpoint= 10.0; //RPM

    public static final double cruiseVelocity= 9.5;
    public static final double acceleration= 10.0; //RPM/s
    public static final double allowedProfileError= 1.0; //RPM
}