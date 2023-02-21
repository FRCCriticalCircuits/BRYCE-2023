// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Auto.DriveStraight;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Triggers.AxisTrigger;
import frc.robot.commands.DriveWithHeading;
import frc.robot.commands.Outtake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SpinUp;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DriveController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PIDContollers;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private DriveController m_driveController = new DriveController(m_driveSubsystem);

  private PIDContollers contollers = new PIDContollers();

  //private ShooterSubsystem shooter = new ShooterSubsystem();

  private Intake intake = new Intake();

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>(); 

  private GenericHID m_DRIVER_GAMEPAD = new GenericHID(Constants.OperatorConstants.DRIVER_GAMEPAD_ID);
  
  private CommandGenericHID m_driverController =
    new CommandGenericHID(Constants.OperatorConstants.DRIVER_GAMEPAD_ID
  );

  private CommandGenericHID m_operatorController = 
    new CommandGenericHID(Constants.OperatorConstants.OPERATOR_GAMEPAD_ID
  );


  public RobotContainer() {
    configureBindings();

    m_driveSubsystem.setDefaultCommand(
      new TeleopDrive(
        m_driveSubsystem,
        m_driveController,
        m_DRIVER_GAMEPAD.getRawAxis(0), 
        m_DRIVER_GAMEPAD.getRawAxis(2), 
        m_DRIVER_GAMEPAD.getRawAxis(1),
        false
      )
    );

    //shooter.setDefaultCommand(new SpinUp(shooter, 40));    

    autoChooser.setDefaultOption("DRIVE STRAIGHT", new DriveStraight(m_driveSubsystem, contollers).driveStaight());


    SmartDashboard.putData("AUTO", autoChooser);
  }

  private Trigger DRIVER_A_BUTTON = new Trigger(m_driverController.button(3));
  private Trigger DRIVER_X_BUTTON = new Trigger(m_driverController.button(4));
  private Trigger DRIVER_LEFT_BUMPER = new Trigger(m_driverController.button(5));
  private Trigger DRIVER_RIGHT_BUMPER = new Trigger(m_driverController.button(6));
  private Trigger DRIVER_RIGHT_TRIGGER = new Trigger(m_driverController.button(8));
  private Trigger DRIVER_LEFT_TRIGGER = new Trigger(m_driverController.button(7));
  private AxisTrigger OPERATOR_X1_STICK = new AxisTrigger(m_operatorController.getRawAxis(0), Constants.OperatorConstants.OPERATOR_X1_THEESHOLD);

  private void configureBindings() {
    DRIVER_A_BUTTON.toggleOnTrue(
      new TeleopDrive(
        m_driveSubsystem,
        m_driveController,
        m_DRIVER_GAMEPAD.getRawAxis(0), 
        m_DRIVER_GAMEPAD.getRawAxis(2), 
        m_DRIVER_GAMEPAD.getRawAxis(1), 
        true
      )
    );

    DRIVER_LEFT_TRIGGER.onTrue(new DriveWithHeading(m_driveSubsystem, 0));
    DRIVER_RIGHT_TRIGGER.onTrue(new RunIntake(intake, DRIVER_RIGHT_TRIGGER));
    DRIVER_RIGHT_BUMPER.onTrue(new Outtake(intake, DRIVER_RIGHT_BUMPER));
    DRIVER_X_BUTTON.onTrue(new InstantCommand(m_driveSubsystem::resetGyro));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}