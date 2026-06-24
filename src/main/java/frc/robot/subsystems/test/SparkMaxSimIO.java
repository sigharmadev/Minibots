package frc.robot.subsystems.test;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SparkMaxSimIO extends MotorIOSparkMax {
    private final DCMotorSim motorSim;
    private final SparkMaxSim sparkSim;

    public SparkMaxSimIO(int canID) {
        super(canID);
        motorSim = new DCMotorSim( LinearSystemId.createDCMotorSystem(
            DCMotor.getCIM(1),TestSparkMaxConstants.MOI,
            TestSparkMaxConstants.GearRatio
            ),
            DCMotor.getCIM(1)
        );

        sparkSim = new SparkMaxSim(cim,DCMotor.getCIM(1)
        );
    }

    @Override
    public void updateInputs(TestSparkIOInputs inputs) {
        double appliedVolts = cim.getAppliedOutput() * RobotController.getBatteryVoltage();

        motorSim.setInputVoltage(appliedVolts);
        motorSim.update(0.02);

        double velocityRPM = Units.radiansPerSecondToRotationsPerMinute(
            motorSim.getAngularVelocityRadPerSec()
        );

        sparkSim.iterate(
            velocityRPM,
            RobotController.getBatteryVoltage(),
            0.02
        );
        inputs.appliedOutput = cim.getAppliedOutput();
        inputs.busVoltage = Volts.of(RobotController.getBatteryVoltage());
        inputs.appliedVolts = Volts.of(appliedVolts);
        inputs.simVelocity = RPM.of(velocityRPM);
        inputs.simCurrent = Amps.of(motorSim.getCurrentDrawAmps());
        super.updateInputs(inputs);
    }
}

  
