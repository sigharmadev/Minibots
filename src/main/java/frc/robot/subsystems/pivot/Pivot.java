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
        Logger.processInputs("Pivot", inputs);
    }

    public void deploy(){
        io.setAngle(PivotConstants.deploySetpoint);
    }

    public void stow(){
        io.setAngle(60);
    }

    public Command deployCommand() {
        return Commands.runOnce(this::deploy, this);
    }

    public Command stowCommand(){
        return Commands.runOnce(this::stow, this);
    }
}
