package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class PivotSimIO extends PivotMotorIO {
    private final DCMotorSim motorSim;
    private final SparkMaxSim sparkSim;

    public PivotSimIO(int canID) {
        super(canID);
        motorSim = new DCMotorSim( LinearSystemId.createDCMotorSystem(
           new DCMotor(12, 24.3, 9.2, 0.25, (104*Math.PI), 1),PivotConstants.MOI,
            PivotConstants.GearRatio
            ),
            new DCMotor(12, 24.3, 9.2, 0.25, (104*Math.PI), 1)
        );

        sparkSim = new SparkMaxSim(cim,new DCMotor(12, 24.3, 9.2, 0.25, (104*Math.PI), 1)
        );
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
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
        super.updateInputs(inputs);
    }
}

  
