// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Auto.AutoBalance;
import frc.robot.Auto.DriveStraight;
import frc.robot.Auto.MiddleAuto;
import frc.robot.Auto.TwoCargoLeftTaxi;
import frc.robot.Auto.TwoCargoRightTaxi;
import frc.robot.Auto.testAuto;
import frc.robot.Util.AutoPIDControllers;
import frc.robot.Util.DriveController;
import frc.robot.Util.GoalType.goalType;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.ManualArmControl;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootWithVision;
import frc.robot.commands.SpinUp;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  
  GenericEntry ShooterVelocitySlider  = Shuffleboard.getTab("Shooter")
    .add("SHOOTER VEL" , 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 80)).getEntry();

  GenericEntry ShooterSpinSlider = Shuffleboard.getTab("Shooter")
    .add("SHOOTER SPIN" , 1).withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1, "max", 1)).getEntry();

  private DriveController m_driveController = new DriveController(m_driveSubsystem);

  private AutoPIDControllers controllers = new AutoPIDControllers();

  private ShooterSubsystem shooter = new ShooterSubsystem();

  private ArmSubsystem armSubsystem = new ArmSubsystem();

  private Sequencer sequencer = new Sequencer();

  private Intake intake = new Intake();

  private LimelightSubsystem limelight = new LimelightSubsystem();

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>(); 

  private GenericHID m_DRIVER_GAMEPAD = new GenericHID(Constants.OperatorConstants.DRIVER_GAMEPAD_ID);
  
  private XboxController m_driverController =
    new XboxController(Constants.OperatorConstants.DRIVER_GAMEPAD_ID
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
        true
      )
    );

    armSubsystem.setDefaultCommand(
      new ManualArmControl(
        armSubsystem, 
        m_operatorController.getRawAxis(0)
      )
    );

    autoChooser.setDefaultOption("NONE", null);
    autoChooser.addOption("DRIVE STRAIGHT", new DriveStraight(m_driveSubsystem, controllers).driveStaight());
    autoChooser.addOption("LEFT TAXI", new TwoCargoLeftTaxi(m_driveSubsystem, shooter, intake, sequencer, m_driveController, controllers).twocargolefttaxi());
    autoChooser.addOption("RIGHT TAXI", new TwoCargoRightTaxi(m_driveSubsystem, shooter, intake, sequencer, controllers).cargorighttaxi());
    autoChooser.addOption("AUTO CLIMB", new AutoBalance(m_driveSubsystem, m_driveController));
    autoChooser.addOption("MIDDLE TAXI", new MiddleAuto(m_driveSubsystem, shooter, sequencer, controllers).middletaxi());
    autoChooser.addOption("Test Auto", new testAuto(m_driveSubsystem, intake, shooter, sequencer).testAuto());

    SmartDashboard.putData("AUTO", autoChooser);
  }

  private Trigger DRIVER_A_BUTTON = new Trigger(m_driverController::getAButton);
  private Trigger DRIVER_X_BUTTON = new Trigger(m_driverController::getXButton);
  private Trigger DRIVER_B_BUTTON = new Trigger(m_driverController::getBButton);
  private Trigger DRIVER_Y_BUTTON = new Trigger(m_driverController::getYButton);
  private Trigger DRIVER_LEFT_BUMPER = new Trigger(m_driverController::getLeftBumper);
  private Trigger DRIVER_RIGHT_BUMPER = new Trigger(m_driverController::getRightBumper);
  private Trigger DRIVER_RIGHT_TRIGGER = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.6);
  private Trigger DRIVER_LEFT_TRIGGER = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.6);
  private Trigger DRIVER_START_BUTTON = new Trigger(m_driverController::getStartButton);
  private Trigger OPERATOR_X_BUTTON = new Trigger(m_operatorController.button(1));

  private void configureBindings() {
    DRIVER_A_BUTTON.toggleOnTrue(
      new TeleopDrive(
        m_driveSubsystem,
        m_driveController,
        m_DRIVER_GAMEPAD.getRawAxis(0), 
        m_DRIVER_GAMEPAD.getRawAxis(2), 
        m_DRIVER_GAMEPAD.getRawAxis(1),
        false
      )
    );

    DRIVER_LEFT_TRIGGER.onTrue(new AlignToTarget(m_driveSubsystem, m_driveController, limelight, DRIVER_LEFT_TRIGGER));
    DRIVER_RIGHT_TRIGGER.debounce(.1).onTrue(new RunIntake(intake, sequencer, false, DRIVER_RIGHT_TRIGGER));
    DRIVER_RIGHT_BUMPER.debounce(.1).onTrue(new RunIntake(intake, sequencer, true, DRIVER_RIGHT_BUMPER));
    DRIVER_START_BUTTON.onTrue(new InstantCommand(m_driveSubsystem::resetHeading));
    DRIVER_Y_BUTTON.debounce(.2).onTrue(new SpinUp(shooter, sequencer, 6, DRIVER_Y_BUTTON));
    DRIVER_X_BUTTON.debounce(.2).onTrue(new ShootWithVision(shooter, sequencer, limelight));
    DRIVER_LEFT_BUMPER.and(DRIVER_Y_BUTTON).onTrue(new InstantCommand(() -> limelight.setGoalType(goalType.HIGH), limelight));
    DRIVER_LEFT_BUMPER.and(DRIVER_X_BUTTON).onTrue(new InstantCommand(() -> limelight.setGoalType(goalType.MID), limelight));
    DRIVER_LEFT_BUMPER.and(DRIVER_A_BUTTON).onTrue(new InstantCommand(() -> limelight.setGoalType(goalType.LOW), limelight));

    OPERATOR_X_BUTTON.toggleOnTrue(new InstantCommand(() -> armSubsystem.engageConeManipulator(), armSubsystem))
      .toggleOnFalse(new InstantCommand(() -> armSubsystem.setConeManipulator(), armSubsystem));
  }
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}