package frc.robot.subsystems.test;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class SparkMaxSimIO extends MotorIOSparkMax {
    DCMotor maxGearBox;
    SparkMaxSim maxSim;

    public SparkMaxSimIO(int CanID) {
        super(CanID);
        maxGearBox = new DCMotor(12, 1.47, 9.2, 0.25,  Units.rotationsPerMinuteToRadiansPerSecond(6000), 1);
        maxSim = new SparkMaxSim(cim, maxGearBox);
    }
    
    @Override
    public void updateInputs(TestSparkIOInputs inputs){
        maxSim.iterate(
            Units.radiansPerSecondToRotationsPerMinute( // motor velocity, in RPM
                maxSim.getVelocity()),
            RoboRioSim.getVInVoltage(), // Simulated battery voltage, in Volts
            0.02); // Time interval, in Seconds

        // SimBattery estimates loaded battery voltages
        // This should include all motors being simulated
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(maxSim.getMotorCurrent()));

        super.updateInputs(inputs);
    }
}
