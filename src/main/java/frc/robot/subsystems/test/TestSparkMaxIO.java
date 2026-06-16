package frc.robot.subsystems.test;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface TestSparkMaxIO {
    @AutoLog

    public static class TestSparkIOInputs{
        public Current motorCurrent= Amps.zero();
        public AngularVelocity motorRPM= edu.wpi.first.units.Units.RPM.zero();
    }

    public default void updateInputs(TestSparkIOInputs inputs) {}

    public default void test(AngularVelocity velocity) {}
}
