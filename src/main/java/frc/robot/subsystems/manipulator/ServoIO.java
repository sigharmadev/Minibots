package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.Servo;

public class ServoIO implements ManipulatorIO {
    public Servo servo;

    public ServoIO(int channel) {
        servo = new Servo(channel);
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.angle= servo.getAngle();
    }

    @Override
    public void setAngle(double angle) {
        servo.setAngle(angle);
    }
}
