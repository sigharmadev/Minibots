
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.test.MotorIOSparkMax;
import frc.robot.subsystems.test.SparkMax;
import frc.robot.subsystems.test.SparkMaxSimIO;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.util.MapleSimUtil;

public class RobotContainer {
    private final CommandXboxController gamepad_ = new CommandXboxController(0);
    private SparkMax testSparkMax;
    private CANBus roborioCANBus = new CANBus();

    public RobotContainer() {
        buildRobot() ;
        createDefaultSubsystems() ;
        
        if (Constants.getRobot() == RobotType.SIMBOT) {
            MapleSimUtil.start();
        }             

        configureBindings();    
    }

    private void configureBindings() {   
      testSparkMax.setDefaultCommand(testSparkMax.testCommand()); 
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private void buildRobot() {
        if (Constants.getMode() != Mode.REPLAY) {
            switch(Constants.getRobot()) {
                case SIMBOT:
                    buildSimBot() ;
                    break ;

                case COMPETITION:
                    buildComp() ;
                    break ;
            }
        }
        else {

        }
    }

    private void buildSimBot() {
      testSparkMax = new SparkMax(new SparkMaxSimIO(2));
    }

    private void buildComp() {
      testSparkMax = new SparkMax(new MotorIOSparkMax(2));      
    }

    private void createDefaultSubsystems() {
      if(testSparkMax == null) {
        testSparkMax= new SparkMax(new MotorIOSparkMax(2));
      }
    }
}

