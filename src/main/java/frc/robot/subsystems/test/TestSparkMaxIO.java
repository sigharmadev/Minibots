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
        public AngularVelocity motorRPM = edu.wpi.first.units.Units.RPM.zero();

        public double appliedOutput = 0.0;
        public Voltage busVoltage = Volts.zero();
        public Voltage appliedVolts = Volts.zero();
        public AngularVelocity simVelocity = edu.wpi.first.units.Units.RPM.zero();
        public Current simCurrent = Amps.zero();
    }

    public default void updateInputs(TestSparkIOInputs inputs) {}

    public default void test(AngularVelocity velocity) {}

    public default void bypass(){}
}
