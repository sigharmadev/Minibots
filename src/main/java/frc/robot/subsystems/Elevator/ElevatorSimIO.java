package frc.robot.subsystems.Elevator;


import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorSimIO extends ElevatorMotorIO {
    private final DCMotorSim motorSim;
    private final SparkMaxSim sparkSim;

    public ElevatorSimIO(int canID, boolean motorInverted, boolean encoderInverted) {
        super(canID, motorInverted, encoderInverted);
        motorSim = new DCMotorSim( LinearSystemId.createDCMotorSystem(
           new DCMotor(12, 24.3, 9.2, 0.25, (104*Math.PI), 1),ElevatorConstants.MOI,
            ElevatorConstants.GearRatio
            ),
            new DCMotor(12, 24.3, 9.2, 0.25, (104*Math.PI), 1)
        );

        sparkSim = new SparkMaxSim(Elevator,new DCMotor(12, 24.3, 9.2, 0.25, (104*Math.PI), 1)
        );
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        double appliedVolts = Elevator.getAppliedOutput() * RobotController.getBatteryVoltage();

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

  
