package frc.robot.subsystems.test;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMax extends SubsystemBase {
    private final TestSparkMaxIO io;
    public final TestSparkIOInputsAutoLogged inputs= new TestSparkIOInputsAutoLogged();

    public SparkMax(TestSparkMaxIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Test", inputs);
        Logger.recordOutput("Test/MotorRPMSetPoint", TestSparkMaxConstants.velocitySetpoint);
    }

    public void test(AngularVelocity velocity) {
        io.test(velocity);
    }

    public Command bypass(){
        return Commands.run(() -> io.bypass(), this);
    }

    public void runVelocitySetpoint() {
        test(TestSparkMaxConstants.velocitySetpoint);
    }

    public void stop() {
        io.test(RPM.of(0));
    }

    public Command testCommand() {
        return Commands.startEnd(() -> runVelocitySetpoint(), this::stop);
    }
}
