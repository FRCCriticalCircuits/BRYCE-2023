package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveController;
import frc.robot.subsystems.DriveSubsystem;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
  private final DriveSubsystem drive;
  private final DriveController driveController;
  private double x1, x2, y;
  private double max;
  private boolean fieldOrientedDrive;

  public TeleopDrive(DriveSubsystem drive, DriveController driveController, double x1, double x2, double y, boolean fieldOrientedDrive) {
    this.drive = drive;
    this.driveController = driveController;
    this.x1 = x1;
    this.x2 = x2;
    this.y = y;
    this.fieldOrientedDrive = fieldOrientedDrive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    max = 0;
  }

  @Override
  public void execute() {
    
    x1 = drive.driverJoystick.getRawAxis(0);
    x2 = drive.driverJoystick.getRawAxis(2);
    y = drive.driverJoystick.getRawAxis(1);
    
    driveController.drive(x1, y, x2, fieldOrientedDrive);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
