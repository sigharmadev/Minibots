package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public class MecanumHardwareIO implements MecanumIO {
    SparkMax motor;
    SparkClosedLoopController motorController;
    RelativeEncoder motorEncoder;
    private AngularVelocity velocityDebug=RPM.of(0);
    private int inputsDebug= 0;

    public MecanumHardwareIO(int CanID, boolean motorInverted) {
        motor= new SparkMax(CanID, MotorType.kBrushed);
        motorController= motor.getClosedLoopController();
        motorEncoder = motor.getEncoder();

        SparkMaxConfig motorConfig= new SparkMaxConfig();

        motorConfig.encoder
            .countsPerRevolution(MecanumConstants.ENCODER_COUNTS_PER_REVOLUTION);
            //.inverted(MecanumConstants.ENCODER_INVERTED);

        motorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(9)
            .closedLoop
                .p(MecanumConstants.kP) 
                .i(MecanumConstants.kI)
                .d(MecanumConstants.kD)
                .feedForward
                    .kS(MecanumConstants.kS)
                    .kV(MecanumConstants.kV)
                    .kA(MecanumConstants.kA);

        motorConfig.closedLoop.maxMotion.
        cruiseVelocity(MecanumConstants.cruiseVelocity)
        .maxAcceleration(MecanumConstants.acceleration);
        // .allowedProfileError(MecanumConstants.allowedProfileError);

        motorConfig.inverted(motorInverted);
        
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }   

    @Override
    public void runVelocity(AngularVelocity velocity){
        Logger.recordOutput("DebugDevVelocity", velocity);
        motorController.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }
    
    @Override
    public void duty(double dutycycle){
        motor.set(dutycycle);
    }

    @Override 
    public void updateInputs(MecanumIOInputs inputs) {
        inputsDebug= 50;
        Logger.recordOutput("InputsDevDebug", inputsDebug);
        inputs.driveAngleRots= motorEncoder.getPosition();
        inputs.driveRotsVelocity= motorEncoder.getVelocity();
        inputs.driveCurrent= Amps.of(motor.getOutputCurrent());
        inputs.appliedVoltage= Volts.of((motor.getAppliedOutput())*(motor.getBusVoltage()));
    }
}
