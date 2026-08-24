package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase{
    private final ManipulatorIO io;
    private final ManipulatorIOInputsAutoLogged inputs = new ManipulatorIOInputsAutoLogged();

    public Manipulator(ManipulatorIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Manipulator", inputs);
    }

    public Command set(double angle){
        return Commands.runOnce(() -> io.setAngle(angle));
    }
}
