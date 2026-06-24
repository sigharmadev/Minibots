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

    // Use a 540/775-sized brushed motor profile to match the goBILDA core physics
    private static final DCMotor goBildaCore = DCMotor.getAndymark9015(1);
    // Combine your internal 19.2 planetary ratio with your external ratio
    private static final double totalGearRatio = 19.2 * MecanumConstants.DriveGearRatio;

    public MecanumSimIO(int canID, boolean motorInverted) {
        // Pass to hardware class constructor (initializes your SparkMax reference)
        super(canID, motorInverted); 

        motorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                goBildaCore, 
                totalGearRatio, 
                MecanumConstants.MOI
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

        // 5. Calculate what the ENCODER sees at the output shaft (divided by gearbox)
        double outputShaftVelocityRPM = motorShaftVelocityRPM / 19.2;
        
        // 6. Directly fill your AdvantageKit inputs with simulated physics data
        inputs.driveRotsVelocity = outputShaftVelocityRPM;
        // Integrate velocity over time (20ms loop) to update simulated position
        inputs.driveAngleRots += (outputShaftVelocityRPM / 60.0) * 0.02;
        inputs.driveCurrent = Amps.of(motorSim.getCurrentDrawAmps());
    }
}
