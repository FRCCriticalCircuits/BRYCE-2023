package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
  private final DriveSubsystem drive;
  private double x1, x2, y;
  private double max;
  private boolean fieldOrientedDrive;

  public TeleopDrive(DriveSubsystem drive, double x1, double x2, double y, boolean fieldOrientedDrive) {
    this.drive = drive;
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
    // ---------------------------
    // SWERVE CALCULATIONS
    // ---------------------------
    
    x1 = drive.driverJoystick.getRawAxis(0);
    x2 = drive.driverJoystick.getRawAxis(2);
    y = drive.driverJoystick.getRawAxis(1);
    

    if(Math.abs(x1) < Constants.OperatorConstants.DRIVER_X1_THRESHOLD){
      x1 = 0;
    }
    if(Math.abs(x2) < Constants.OperatorConstants.DRIVER_Y1_THRESHOLD){
      x2 = 0;
    }
    if(Math.abs(y) < Constants.OperatorConstants.DRIVER_X2_THRESHOLD){
      y = 0;
    }

    //double magnitude = Math.sqrt(Math.pow(x1, 2) + Math.pow(y, 2));

    double a = x1 - x2 * (Constants.PhysicalConstants.SIDE_LENGTH / Constants.PhysicalConstants.SIDE_TO_CORNER );
    double b = x1 + x2 * (Constants.PhysicalConstants.SIDE_LENGTH / Constants.PhysicalConstants.SIDE_TO_CORNER);
    double c = y - x2 * (Constants.PhysicalConstants.SIDE_WIDTH / Constants.PhysicalConstants.SIDE_TO_CORNER);
    double d = y + x2 * (Constants.PhysicalConstants.SIDE_WIDTH / Constants.PhysicalConstants.SIDE_TO_CORNER);

    double frontleftspeed = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));
    double frontrightspeed = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
    double rearleftspeed = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));
    double rearrightspeed = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));

    if (max > 1) {
      frontleftspeed /= max;
      frontrightspeed /= max;
      rearrightspeed /= max;
      rearleftspeed /= max;
    }

    double frontleftangle = Math.atan2(b, c) * 180 / Math.PI;
    double frontrightangle = Math.atan2(b, d) * 180 / Math.PI;
    double rearleftangle = Math.atan2(a, c) * 180 / Math.PI;
    double rearrightangle = Math.atan2(a, d) * 180 / Math.PI;

    // --------------------------

    // ------------------------------------------
    //  SET WHEEL ANGLE AND SPEED USING INPUT
    // ------------------------------------------
    
      drive.frontLeft.setSpeed(frontleftspeed);
      drive.frontRight.setSpeed(frontrightspeed);        
      drive.rearLeft.setSpeed(rearleftspeed);
      drive.rearRight.setSpeed(rearrightspeed);

      if(fieldOrientedDrive){
        drive.frontLeft.setAngle(frontleftangle - drive.getHeading().getDegrees());
      }else{
        drive.frontLeft.setAngle(frontleftangle);
      }
      if(fieldOrientedDrive){
        drive.frontRight.setAngle(frontrightangle - drive.getHeading().getDegrees());
      }else{
        drive.frontRight.setAngle(frontrightangle);
      }
      if(fieldOrientedDrive){
        drive.rearLeft.setAngle(rearleftangle - drive.getHeading().getDegrees());
      }else{
        drive.rearLeft.setAngle(rearleftangle);
      }
      if(fieldOrientedDrive){
        drive.rearRight.setAngle(rearrightangle - drive.getHeading().getDegrees());
      }else{
        drive.rearRight.setAngle(rearrightangle);
      }

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
