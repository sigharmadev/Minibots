
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
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
import frc.robot.subsystems.drive.MecanumSimIO;
import frc.robot.subsystems.drive.NavXIO;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.PivotMotorIO;
import frc.robot.subsystems.pivot.PivotSimIO;
import frc.robot.subsystems.test.MotorIOSparkMax;
import frc.robot.subsystems.test.SparkMax;
import frc.robot.subsystems.test.SparkMaxSimIO;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.commands.AutoCommands;
import frc.robot.util.MapleSimUtil;

public class RobotContainer {
    private LoggedDashboardChooser<Command> autoChooser_ = new LoggedDashboardChooser<>("Auto choices");
    private final CommandXboxController gamepad_ = new CommandXboxController(0);
    private SparkMax testSparkMax;
    private Drive drive;
    private Pivot pivot;

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

        autoChooser_ = new LoggedDashboardChooser<>("Auto Choices");
        autoChooser_.addDefaultOption("Test Auto", AutoCommands.test(drive));
        autoChooser_.addOption("RedSideClose", AutoCommands.redSideClose(drive, testSparkMax));
        autoChooser_.addOption("RedSideFar", AutoCommands.redSide(drive, testSparkMax));
        autoChooser_.addOption("BlueSideRight", AutoCommands.blueSide(drive, testSparkMax));
        configureBindings();   
        configureDriveBindings(); 

    }

    private void configureBindings() {   
      if(pivot.inputs.motorAngle==(PivotConstants.deploySetpoint-5.0)||pivot.inputs.motorAngle==(PivotConstants.deploySetpoint+5.0)){
        gamepad_.a().onTrue(pivot.stowCommand());
      } else{
        gamepad_.a().onTrue(pivot.deployCommand());
      }
    }

    private void configureDriveBindings(){
      drive.setDefaultCommand(DriveCommands.joystickDriveField().withName("JoystickDrive"));
      gamepad_.y().onTrue(drive.zeroGyro());
      gamepad_.b().onTrue(drive.zeroPose());
    }


    public Command getAutonomousCommand() {
        return autoChooser_.get();
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
      drive= new Drive(new NavXIO());
      pivot= new Pivot(new PivotSimIO(6, false, false));
    }

    private void buildComp() {
      drive= new Drive(new NavXIO());
      pivot= new Pivot(new PivotMotorIO(6, false, false));
    }

    private void createDefaultSubsystems() {
      if(pivot==null){
        pivot= new Pivot(new PivotMotorIO(6, false, false));
      }
      if(drive==null){
        drive= new Drive(new NavXIO());
      }
    }
}

