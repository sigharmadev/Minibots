package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.units.measure.AngularVelocity;

public class ElevatorConstants {
    public static final int ENCODER_COUNTS_PER_REVOLUTION = 538;

    public static final double kP= 0.3;
    public static final double kI= 0.0001;
    public static final double kD= 0.05;
    public static final double kS= 0.0;
    public static final double kV= 0.0;
    public static final double kA= 0.0;

    
    public static final double cruiseVelocity= 110;
    public static final double acceleration= 150; //RPM/s
    public static final double allowedProfileError= 0.0; //RPM

    public static final double MOI= 0.01;
    public static final double GearRatio= 1.0;

    public static final double deploySetpoint= 50.0; //rotations
}