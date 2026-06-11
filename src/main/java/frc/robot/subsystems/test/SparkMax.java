package frc.robot.subsystems.test;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMax extends SubsystemBase {
    private final TestSparkMaxIO io;
    public final TestSparkIOInputs inputs= new TestSparkIOInputs();

    public SparkMax(TestSparkMaxIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Test", inputs);
        Logger.recordOutput("Test/MotorRPMSetPoint", TestSparkMaxConstants.velocitySetpoint);
    }

    public void test() {
        io.test();
    }
}
