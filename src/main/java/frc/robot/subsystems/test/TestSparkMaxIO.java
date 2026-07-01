package frc.robot.subsystems.test;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface TestSparkMaxIO {
    @AutoLog

    public static class TestSparkIOInputs{
        public Current motorCurrent = Amps.zero();
        public double motorRPM = 0.0;
        public Voltage voltageApplied= Volts.of(0.0);
    }

    public default void updateInputs(TestSparkIOInputs inputs) {}

    public default void test(AngularVelocity velocity) {}

    public default void bypass(double dutycycle){}
}
