package frc.robot.subsystems.test;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class MotorIOSparkMax implements TestSparkMaxIO {
    SparkMax cim;
    SparkClosedLoopController cimController;

    public MotorIOSparkMax() {
        cim = new SparkMax(2, MotorType.kBrushed);
        cimController = cim.getClosedLoopController();

        SparkMaxConfig cimConfig= new SparkMaxConfig();

        cimConfig.encoder.countsPerRevolution(TestSparkMaxConstants.ENCODER_COUNTS_PER_REVOLUTION)
        .inverted(TestSparkMaxConstants.ENCODER_INVERTED);

        /*cimConfig.closedLoop.
        p(TestSparkMaxConstants.kP)
        .i(TestSparkMaxConstants.kI)
        .d(TestSparkMaxConstants.kD)
        .outputRange(-10.0, 10.0);   

        cimConfig.closedLoop.feedForward.
        kS(TestSparkMaxConstants.kS)
        .kV(TestSparkMaxConstants.kV)
        .kA(TestSparkMaxConstants.kA);*/

        cimConfig.closedLoop.maxMotion.
        cruiseVelocity(TestSparkMaxConstants.cruiseVelocity)
        .maxAcceleration(TestSparkMaxConstants.acceleration)
        .allowedProfileError(TestSparkMaxConstants.allowedProfileError);

        cimConfig.signals
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .outputCurrentPeriodMs(20);

        
        cim.configure(cimConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void test(){
        cimController.setSetpoint(TestSparkMaxConstants.velocitySetpoint, ControlType.kVelocity);
    }

    @Override 
    public void updateInputs(TestSparkIOInputsAutoLogged inputs) {
        inputs.motorRpm = cim.getEncoder().getVelocity();
        inputs.motorCurrent = cim.getOutputCurrent();
    }
}
