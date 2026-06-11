package frc.robot.subsystems.test;

import org.littletonrobotics.junction.AutoLog;

public interface TestSparkMaxIO {
    @AutoLog

    public static class TestSparkIOInputsAutoLogged{
        public double motorRpm= 0.0;
        public double motorCurrent= 0.0;
    }

    public default void updateInputs(TestSparkIOInputsAutoLogged inputs) {}
}
