package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Servo;


public interface ManipulatorIO {
    @AutoLog
    public static class ManipulatorIOInputs{
        public double angle= 0.0;
    }

    public default void updateInputs(ManipulatorIOInputs inputs) {}
    public default void setAngle(double angle) {}
}
