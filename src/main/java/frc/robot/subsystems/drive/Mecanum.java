package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mecanum extends SubsystemBase {
    private final MecanumIO io;
    public final MecanumIOInputsAutoLogged inputs= new MecanumIOInputsAutoLogged();

    public Mecanum(MecanumIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Test", inputs);
    }

    public void setVelocity(AngularVelocity velocity) {
        io.runVelocity(velocity);
    }

    public void setSetpoint(MecanumDriveWheelSpeeds speeds){
        double driveRads= (speeds/(MecanumConstants.wheelRadius)).getValueAsDouble();
        setVelocity(RotationsPerSecond.of(driveRads));
    }

    public Command runVelocity(MecanumDriveWheelSpeeds speeds){
        return Commands.run(()-> setSetpoint(speeds));
    }

    public double getWheelPositions(){
        double positionMeters= inputs.driveAngleRads*MecanumConstants.wheelRadius;
        return positionMeters;
    }

    public void stop(){
        setVelocity(RotationsPerSecond.of(0));
    }
    
}
