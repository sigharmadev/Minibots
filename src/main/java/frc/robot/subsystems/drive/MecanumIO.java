package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface MecanumIO {
    @AutoLog
    public static class MecanumIOInputs{
        public double driveRotsVelocity= 0.0;
        public double driveAngleRots= 0.0;
        public Current driveCurrent= Amps.of(0);
        public Voltage appliedVoltage= Volts.of(0);
    }

    public default void updateInputs(MecanumIOInputs inputs) {}

    public default void runVelocity(AngularVelocity velocity) {}

    public default void bypass(double dutycycle){}
}
