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
    private final String name;

    private AngularVelocity velocitySetpoint;

    public Mecanum(MecanumIO io, String name) {
        this.io = io;
        this.name= name;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Mecanum/" + name, inputs);
        Logger.recordOutput("Velocity/Setpoint" + name, velocitySetpoint);
    }

    public void setVelocity(AngularVelocity velocity) {
        velocitySetpoint= velocity;
        io.runVelocity(velocity);
    }

    public void setSetpoint(double speeds){
        double driveRads= speeds/(MecanumConstants.wheelRadius);
        double driveRPM= (driveRads/(2.0 * Math.PI))*60;
        //Drive.debug= 1000;
        //Logger.recordOutput("DebugDev", Drive.debug);
        setVelocity(RPM.of(driveRPM));
    }

    public Command runVelocity(double speed){
        return Commands.run(()-> setSetpoint(speed));
    }

    public double getWheelPositions(){
        double positionMeters= inputs.driveAngleRots*2*MecanumConstants.wheelRadius*Math.PI;
        return positionMeters;
    }

    public void stop(){
        setVelocity(RPM.of(0));
    }

    public void bypass(double dutycycle){
        io.bypass(dutycycle);
    }
}
