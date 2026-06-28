package frc.robot.subsystems.drive;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.units.measure.Current;
import static edu.wpi.first.units.Units.Amps;

public class MecanumSimIO extends MecanumHardwareIO {
    private final DCMotorSim motorSim;
    private final SparkMaxSim sparkSim;

    //Create new dc motor object based on specifications of exact motor 
    private static final DCMotor goBildaCore = new DCMotor(12, 24.3, 9.2, 0.25, (104*Math.PI), 1);

    public MecanumSimIO(int canID, boolean motorInverted) {
        // Pass to hardware class constructor (initializes your SparkMax reference)
        super(canID, motorInverted); 

        motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                goBildaCore, MecanumConstants.MOI,
                MecanumConstants.DriveGearRatio
            ),
            goBildaCore
        );
        
        sparkSim = new SparkMaxSim(motor, goBildaCore);
    }

    @Override
    public void updateInputs(MecanumIOInputs inputs) {
        // 1. Read how many volts the software is trying to send to the motor
        double appliedVolts = motor.getAppliedOutput() * RobotController.getBatteryVoltage();
        
        // 2. Feed that voltage into the physics engine and step forward 20ms
        motorSim.setInputVoltage(appliedVolts);
        motorSim.update(0.02);

        // 3. Get the motor's shaft speed in RPM *before* the gearbox reduction
        double motorShaftVelocityRPM = Units.radiansPerSecondToRotationsPerMinute(
            motorSim.getAngularVelocityRadPerSec()
        );

        // 4. Update the REV Simulation wrapper so internal PID algorithms work
        sparkSim.iterate(motorShaftVelocityRPM, RobotController.getBatteryVoltage(), 0.02);
        
        super.updateInputs(inputs);
    }
}
