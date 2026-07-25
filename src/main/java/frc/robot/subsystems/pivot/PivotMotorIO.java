package frc.robot.subsystems.pivot;

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

public class PivotMotorIO implements PivotIO {
    SparkMax pivot;
    SparkClosedLoopController pivotController;
    RelativeEncoder pivotEncoder;

    public PivotMotorIO(int CanID, boolean motorInverted, boolean encoderInverted) {
        pivot = new SparkMax(CanID, MotorType.kBrushed);
        pivotController = pivot.getClosedLoopController();
        pivotEncoder = pivot.getEncoder();

        SparkMaxConfig pivotConfig= new SparkMaxConfig();

        pivotConfig.encoder.countsPerRevolution(PivotConstants.ENCODER_COUNTS_PER_REVOLUTION)
        .inverted(encoderInverted);
        pivotConfig.closedLoop.
        p(PivotConstants.kP)
        .i(PivotConstants.kI)
        .d(PivotConstants.kD); 

        pivotConfig.closedLoop.feedForward.
        kS(PivotConstants.kS)
        .kV(PivotConstants.kV)
        .kA(PivotConstants.kA);

        pivotConfig.closedLoop.maxMotion.
        cruiseVelocity(PivotConstants.cruiseVelocity)
        .maxAcceleration(PivotConstants.acceleration)
        .allowedProfileError(PivotConstants.allowedProfileError);

        pivotConfig.inverted(motorInverted);

        pivotConfig.smartCurrentLimit(9);
         
        pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }



    @Override 
    public void updateInputs(PivotIOInputs inputs) {
        inputs.motorRPM = pivotEncoder.getVelocity();
        inputs.motorCurrent = Amps.of(pivot.getOutputCurrent());
        inputs.voltageApplied= Volts.of((pivot.getAppliedOutput())*(pivot.getBusVoltage()));
        inputs.motorAngle= pivotEncoder.getPosition();
    }

    @Override
    public void setAngle(double setpoint) {
        pivotController.setSetpoint(setpoint, ControlType.kPosition);
    }
}
