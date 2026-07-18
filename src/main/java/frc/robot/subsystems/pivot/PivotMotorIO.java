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
    SparkMax cim;
    SparkClosedLoopController cimController;
    RelativeEncoder cimEncoder;

    public PivotMotorIO(int CanID) {
        cim = new SparkMax(CanID, MotorType.kBrushed);
        cimController = cim.getClosedLoopController();
        cimEncoder = cim.getEncoder();

        SparkMaxConfig cimConfig= new SparkMaxConfig();

        cimConfig.encoder.countsPerRevolution(PivotConstants.ENCODER_COUNTS_PER_REVOLUTION)
        .inverted(PivotConstants.ENCODER_INVERTED);

        cimConfig.closedLoop.
        p(PivotConstants.kP)
        .i(PivotConstants.kI)
        .d(PivotConstants.kD);

        cimConfig.closedLoop.feedForward.
        kS(PivotConstants.kS)
        .kV(PivotConstants.kV)
        .kA(PivotConstants.kA);

        cimConfig.closedLoop.maxMotion.
        cruiseVelocity(PivotConstants.cruiseVelocity)
        .maxAcceleration(PivotConstants.acceleration)
        .allowedProfileError(PivotConstants.allowedProfileError);

        cimConfig.inverted(true);

        cimConfig.smartCurrentLimit(9);
         
        cim.configure(cimConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void test(AngularVelocity velocity){
        cimController.setSetpoint(velocity.in(RPM), ControlType.kMAXMotionVelocityControl);
    }
    @Override
    public void bypass(double dutycycle){
        cim.set(dutycycle);
    }

    @Override 
    public void updateInputs(PivotIOInputs inputs) {
        inputs.motorRPM = cimEncoder.getVelocity();
        inputs.motorCurrent = Amps.of(cim.getOutputCurrent());
        inputs.voltageApplied= Volts.of((cim.getAppliedOutput())*(cim.getBusVoltage()));
    }
}
