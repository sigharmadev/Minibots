package frc.robot.subsystems.manipulator;
import org.littletonrobotics.junction.AutoLog;



public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs{
        public double angle= 0.0;
    }

    public default void updateInputs(ManipulatorIOInputs inputs) {}
    public default void setAngle(double angle) {}
}
