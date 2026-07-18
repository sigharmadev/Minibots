package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {
    private final PivotIO io;
    public final PivotIOInputsAutoLogged inputs= new PivotIOInputsAutoLogged();

    public Pivot(PivotIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Test", inputs);
        Logger.recordOutput("Test/MotorRPMSetPoint", PivotConstants.velocitySetpoint);
    }

    public void test(AngularVelocity velocity) {
        io.test(velocity);
    }

    public Command bypass(double dutycycle){
        return Commands.startEnd(() -> io.bypass(dutycycle), this::stop);
    }

    public void runVelocitySetpoint() {
        test(PivotConstants.velocitySetpoint);
    }

    public void stop() {
        io.test(RPM.of(0));
    }

    public Command testCommand() {
        return Commands.startEnd(() -> runVelocitySetpoint(), this::stop);
    }
}
