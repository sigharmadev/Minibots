
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveCommands;
import frc.robot.subsystems.drive.NavXIO;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ServoIO;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorMotorIO;
import frc.robot.subsystems.Elevator.ElevatorSimIO;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.commands.AutoCommands;
import frc.robot.util.MapleSimUtil;

public class RobotContainer {
    private LoggedDashboardChooser<Command> autoChooser_ = new LoggedDashboardChooser<>("Auto choices");
    private final CommandXboxController gamepad_ = new CommandXboxController(0);
    private Drive drive;
    private Elevator elevator;
    private Manipulator manipulator;

    public RobotContainer() {
        buildRobot();
        createDefaultSubsystems();
        
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
        autoChooser_.addOption("Drive Forward", AutoCommands.forward(drive));
        autoChooser_.addOption("Strafe ", AutoCommands.strafe(drive));
        configureBindings();   
        configureDriveBindings(); 

    }

    private void configureBindings() {   
      gamepad_.x().onTrue(elevator.l3Place());
      gamepad_.leftTrigger().and(gamepad_.rightTrigger()).onTrue(elevator.set(elevator.inputs.motorAngle-2.0));
      gamepad_.rightTrigger().onTrue(elevator.reset());
    }

    private void configureDriveBindings(){
      drive.setDefaultCommand(DriveCommands.joystickDriveField().withName("JoystickDrive"));
      gamepad_.y().onTrue(drive.zeroGyro());
      gamepad_.povLeft().onTrue(drive.zeroPose());
      
      //Set of field relative commands to control the robot linearly at a slower speed
      gamepad_.povUp().whileTrue(DriveCommands.slowDriveField(drive, ()->0.25, ()->0.0, ()->0.0));
      gamepad_.povDown().whileTrue(DriveCommands.slowDriveField(drive, ()->-0.25, ()->0.0, ()->0.0));
      gamepad_.povLeft().whileTrue(DriveCommands.slowDriveField(drive, ()->0.0, ()->0.25, ()->0.0));
      gamepad_.povRight().whileTrue(DriveCommands.slowDriveField(drive, ()->0.0, ()->-0.25, ()->0.0));
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
      elevator= new Elevator(new ElevatorSimIO(6, false, false));
    }

    private void buildComp() {
      drive= new Drive(new NavXIO());
      elevator= new Elevator(new ElevatorMotorIO(6, false, false));
      manipulator= new Manipulator(new ServoIO(0));
    }

    private void createDefaultSubsystems() {
      if(elevator==null){
        elevator= new Elevator(new ElevatorMotorIO(6, false, false));
      }
      if(drive==null){
        drive= new Drive(new NavXIO());
      }
      if(manipulator==null){
        manipulator= new Manipulator(new ServoIO(0));
      }
    }
}

