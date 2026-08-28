package frc.robot.subsystems.Elevator;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    public final ElevatorIOInputsAutoLogged inputs= new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void setAngle(double setpoint) {
        io.setAngle(setpoint);
    }

    public Command l3Place(){
        return Commands.runOnce(() -> setAngle(ElevatorConstants.l3));
    }

    public Command reset(){
        return Commands.runOnce(() -> setAngle(ElevatorConstants.reset));
    }

    public Command set(double setpoint){
        return Commands.runOnce(() -> setAngle(setpoint));
    }
}
