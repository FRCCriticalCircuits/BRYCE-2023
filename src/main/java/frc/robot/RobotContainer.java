// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.Auto.DriveStraight30;
import frc.robot.subsystems.Drive.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private Command autonomouseCommand;

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>(); 

  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.DRIVER_GAMEPAD_ID);

  public RobotContainer() {
    configureBindings();

    m_driveSubsystem.setDefaultCommand(
      new TeleopDrive(
        m_driveSubsystem, 
        m_driverController.getRawAxis(0), 
        m_driverController.getRawAxis(2), 
        m_driverController.getRawAxis(1)
      )
    );

    autoChooser.setDefaultOption("DRIVE STRAIGHT 30", new DriveStraight30(m_driveSubsystem));

    SmartDashboard.putData("AUTO", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}