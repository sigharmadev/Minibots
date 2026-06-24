package frc.robot.subsystems.test;

import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.AngularVelocity;

public class TestSparkMaxConstants {
    public static final int CIM_MOTOR_ID = 2;
    public static final int ENCODER_COUNTS_PER_REVOLUTION = 2150;
    public static final boolean ENCODER_INVERTED = true;

    public static final double kP= 0.001;
    public static final double kI= 0.0;
    public static final double kD= 0.0;
    public static final double kS= 0.01;
    public static final double kV= 0.001;
    public static final double kA= 0.0;

    public static final AngularVelocity velocitySetpoint= RPM.of(200.0); //RPM

    public static final double cruiseVelocity= 75;
    public static final double acceleration= 100; //RPM/s
    public static final double allowedProfileError= 3.0; //RPM

    public static final double MOI= 0.01;
    public static final double GearRatio= 1.0;
}