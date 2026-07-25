package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface PivotIO {
    @AutoLog

    public static class PivotIOInputs{
        public Current motorCurrent = Amps.zero();
        public double motorAngle= 0.0;
        public double motorRPM = 0.0;
        public Voltage voltageApplied= Volts.of(0.0);
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setAngle(double setpoint){}
}
