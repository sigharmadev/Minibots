package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ElevatorIO {
    @AutoLog

    public static class ElevatorIOInputs{
        public Current motorCurrent = Amps.zero();
        public double motorAngle= 0.0;
        public double motorRPM = 0.0;
        public Voltage voltageApplied= Volts.of(0.0);
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setAngle(double setpoint){}
}
