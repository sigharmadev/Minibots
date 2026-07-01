package frc.robot.subsystems.test;

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

public class MotorIOSparkMax implements TestSparkMaxIO {
    SparkMax cim;
    SparkClosedLoopController cimController;
    RelativeEncoder cimEncoder;

    public MotorIOSparkMax(int CanID) {
        cim = new SparkMax(CanID, MotorType.kBrushed);
        cimController = cim.getClosedLoopController();
        cimEncoder = cim.getEncoder();

        SparkMaxConfig cimConfig= new SparkMaxConfig();

        cimConfig.encoder.countsPerRevolution(TestSparkMaxConstants.ENCODER_COUNTS_PER_REVOLUTION)
        .inverted(TestSparkMaxConstants.ENCODER_INVERTED);

        cimConfig.closedLoop.
        p(TestSparkMaxConstants.kP)
        .i(TestSparkMaxConstants.kI)
        .d(TestSparkMaxConstants.kD);

        cimConfig.closedLoop.feedForward.
        kS(TestSparkMaxConstants.kS)
        .kV(TestSparkMaxConstants.kV)
        .kA(TestSparkMaxConstants.kA);

        cimConfig.closedLoop.maxMotion.
        cruiseVelocity(TestSparkMaxConstants.cruiseVelocity)
        .maxAcceleration(TestSparkMaxConstants.acceleration)
        .allowedProfileError(TestSparkMaxConstants.allowedProfileError);

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
    public void updateInputs(TestSparkIOInputs inputs) {
        inputs.motorRPM = cimEncoder.getVelocity();
        inputs.motorCurrent = Amps.of(cim.getOutputCurrent());
        inputs.voltageApplied= Volts.of((cim.getAppliedOutput())*(cim.getBusVoltage()));
    }
}
