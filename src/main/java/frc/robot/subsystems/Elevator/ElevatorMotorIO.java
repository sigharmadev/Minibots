package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;

public class ElevatorMotorIO implements ElevatorIO {
    SparkMax Elevator;
    SparkClosedLoopController ElevatorController;
    RelativeEncoder ElevatorEncoder;

    public ElevatorMotorIO(int CanID, boolean motorInverted, boolean encoderInverted) {
        Elevator = new SparkMax(CanID, MotorType.kBrushed);
        ElevatorController = Elevator.getClosedLoopController();
        ElevatorEncoder = Elevator.getEncoder();

        SparkMaxConfig ElevatorConfig= new SparkMaxConfig();

        ElevatorConfig.encoder.countsPerRevolution(ElevatorConstants.ENCODER_COUNTS_PER_REVOLUTION)
        .inverted(encoderInverted);
        ElevatorConfig.closedLoop.
        p(ElevatorConstants.kP)
        .i(ElevatorConstants.kI)
        .d(ElevatorConstants.kD); 

        ElevatorConfig.closedLoop.feedForward.
        kS(ElevatorConstants.kS)
        .kV(ElevatorConstants.kV)
        .kA(ElevatorConstants.kA);

        ElevatorConfig.closedLoop.maxMotion.
        cruiseVelocity(ElevatorConstants.cruiseVelocity)
        .maxAcceleration(ElevatorConstants.acceleration)
        .allowedProfileError(ElevatorConstants.allowedProfileError);

        ElevatorConfig.inverted(motorInverted);

        ElevatorConfig.smartCurrentLimit(9);
         
        Elevator.configure(ElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }



    @Override 
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.motorRPM = ElevatorEncoder.getVelocity();
        inputs.motorCurrent = Amps.of(Elevator.getOutputCurrent());
        inputs.voltageApplied= Volts.of((Elevator.getAppliedOutput())*(Elevator.getBusVoltage()));
        inputs.motorAngle= ElevatorEncoder.getPosition();
    }

    @Override
    public void setAngle(double setpoint) {
        ElevatorController.setSetpoint(setpoint, ControlType.kPosition);
    }
}
