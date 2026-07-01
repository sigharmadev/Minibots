
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
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOMaple;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.MecanumSimIO;
import frc.robot.subsystems.test.MotorIOSparkMax;
import frc.robot.subsystems.test.SparkMax;
import frc.robot.subsystems.test.SparkMaxSimIO;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.util.MapleSimUtil;

public class RobotContainer {
    private final CommandXboxController gamepad_ = new CommandXboxController(0);
    private SparkMax testSparkMax;
    private Drive drive;
    private SparkMax secondTestSparkMax;
    private CANBus roborioCANBus = new CANBus();

    public RobotContainer() {
        buildRobot() ;
        createDefaultSubsystems() ;
        
        if (Constants.getRobot() == RobotType.SIMBOT) {
            MapleSimUtil.start();
        }             
        
        DriveCommands.configure(
            drive,
            () -> -gamepad_.getLeftY(),
            () -> -gamepad_.getLeftX(),
            () -> -gamepad_.getRightX()
        );

        drive.configure(
          () -> -gamepad_.getRightX()
        );
        configureBindings();   
        configureDriveBindings(); 

    }

    private void configureBindings() {   
      gamepad_.leftTrigger().whileTrue(testSparkMax.bypass(0.5));
      gamepad_.rightTrigger().whileTrue(testSparkMax.bypass(-0.5));
    }

    private void configureDriveBindings(){
      /*gamepad_.povDown().whileTrue(drive.runLinearCmd(MetersPerSecond.of(0.4), MetersPerSecond.of(0),
        RadiansPerSecond.of(0)));*/
      drive.setDefaultCommand(DriveCommands.joystickDrive().withName("JoystickDrive"));
      //drive.setDefaultCommand(DriveCommands.turn());
      /*gamepad_.povUp().whileTrue(drive.runLinearCmd(MetersPerSecond.of(-0.4), MetersPerSecond.of(0), 
      RadiansPerSecond.of(0))); */
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
      testSparkMax = new SparkMax(new SparkMaxSimIO( 6));
      drive= new Drive();
    }

    private void buildComp() {
      testSparkMax = new SparkMax(new MotorIOSparkMax(6));    
      drive= new Drive();
    }

    private void createDefaultSubsystems() {
      if(testSparkMax == null) {
        testSparkMax= new SparkMax(new MotorIOSparkMax(6));
      }
    }
}

