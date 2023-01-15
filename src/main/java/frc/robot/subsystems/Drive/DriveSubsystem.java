// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    public Module frontLeft = new Module(
        Constants.MotorIDs.FRONT_LEFT_FORWARD_ID,
        Constants.MotorIDs.FRONT_LEFT_ROTATION_ID
    );

    public Module frontRight = new Module(
        Constants.MotorIDs.FRONT_RIGHT_FORWARD_ID, 
        Constants.MotorIDs.FRONT_RIGHT_ROTATION_ID
    );

    public Module rearLeft = new Module(
        Constants.MotorIDs.REAR_LEFT_FORWARD_ID, 
        Constants.MotorIDs.REAR_LEFT_ROTATION_ID
    );

    public Module rearRight = new Module(
        Constants.MotorIDs.REAR_RIGHT_FORWARD_ID, 
        Constants.MotorIDs.REAR_RIGHT_ROTATION_ID
    );

    public Joystick driverJoystick = new Joystick(0);

    private AHRS gyro = new AHRS();

    private Translation2d frontleftLocation = new Translation2d(Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2));
    private Translation2d frontrightLocation = new Translation2d(Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), -1 * Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2));
    private Translation2d rearleftLocation = new Translation2d(-1 * Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2));
    private Translation2d rearrightLocation = new Translation2d(-1 * Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), -1 * Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2));

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontleftLocation, 
      frontrightLocation, 
      rearleftLocation, 
      rearrightLocation
    );

    private SwerveModulePosition[] modulePositions = {
      new SwerveModulePosition(frontLeft.getDistance(), frontLeft.getRotation2d()),
      new SwerveModulePosition(frontRight.getDistance(), frontRight.getRotation2d()),
      new SwerveModulePosition(rearLeft.getDistance(), rearLeft.getRotation2d()),
      new SwerveModulePosition(rearRight.getDistance(), rearRight.getRotation2d())
    };

    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getHeading(), modulePositions);

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, getHeading(), modulePositions, new Pose2d());
    
  public DriveSubsystem() {
    poseEstimator.update(getHeading(), modulePositions);
    odometry.update(getHeading(), modulePositions);
    OutputModuleInfo();
    setup();
  }

  public void setup(){

    //frontRight.setInverted(true);
    //rearRight.setInverted(true);

    // ------------------
    // SET PID VALUES
    // ------------------

    


    frontLeft.setForwardPID(
        Constants.PIDConstants.FRONT_LEFT_FORWARD_PID0_P, 
        Constants.PIDConstants.FRONT_LEFT_FORWARD_PID0_I, 
        Constants.PIDConstants.FRONT_LEFT_FORWARD_PID0_D, 
        Constants.PIDConstants.FRONT_LEFT_FORWARD_PID0_FF, 0
    );

    frontLeft.setTurnPID(
        Constants.PIDConstants.FRONT_LEFT_ROTATION_PID0_P, 
        Constants.PIDConstants.FRONT_LEFT_ROTATION_PID0_I, 
        Constants.PIDConstants.FRONT_LEFT_ROTATION_PID0_D, 
        Constants.PIDConstants.FRONT_LEFT_ROTATION_PID0_FF, 0
    );

    frontRight.setForwardPID(
        Constants.PIDConstants.FRONT_RIGHT_FORWARD_PID0_P, 
        Constants.PIDConstants.FRONT_RIGHT_FORWARD_PID0_I, 
        Constants.PIDConstants.FRONT_RIGHT_FORWARD_PID0_D, 
        Constants.PIDConstants.FRONT_RIGHT_FORWARD_PID0_FF, 0
    );

    frontRight.setTurnPID(
        Constants.PIDConstants.FRONT_RIGHT_FORWARD_PID0_P, 
        Constants.PIDConstants.FRONT_RIGHT_ROTATION_PID0_I, 
        Constants.PIDConstants.FRONT_RIGHT_ROTATION_PID0_D, 
        Constants.PIDConstants.FRONT_RIGHT_ROTATION_PID0_FF, 0
    );

    rearLeft.setForwardPID(
        Constants.PIDConstants.REAR_LEFT_FORWARD_PID0_P, 
        Constants.PIDConstants.REAR_LEFT_FORWARD_PID0_I, 
        Constants.PIDConstants.REAR_LEFT_FORWARD_PID0_D, 
        Constants.PIDConstants.REAR_LEFT_FORWARD_PID0_FF, 0
    );

    rearLeft.setTurnPID(
        Constants.PIDConstants.REAR_LEFT_ROTATION_PID0_P, 
        Constants.PIDConstants.REAR_LEFT_ROTATION_PID0_I, 
        Constants.PIDConstants.REAR_LEFT_ROTATION_PID0_D, 
        Constants.PIDConstants.REAR_LEFT_ROTATION_PID0_FF, 0
    );

    rearRight.setForwardPID(
        Constants.PIDConstants.REAR_RIGHT_FORWARD_PID0_P, 
        Constants.PIDConstants.REAR_RIGHT_FORWARD_PID0_I, 
        Constants.PIDConstants.REAR_RIGHT_FORWARD_PID0_D, 
        Constants.PIDConstants.REAR_RIGHT_FORWARD_PID0_FF, 0
    );

    rearRight.setTurnPID(
        Constants.PIDConstants.REAR_RIGHT_ROTATION_PID0_P, 
        Constants.PIDConstants.REAR_RIGHT_ROTATION_PID0_I, 
        Constants.PIDConstants.REAR_RIGHT_ROTATION_PID0_D, 
        Constants.PIDConstants.REAR_RIGHT_ROTATION_PID0_FF, 0
    );
  }
/*
  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
 */

  public void OutputModuleInfo() {
    SmartDashboard.getNumber("FRONT LEFT ANGLE", frontLeft.getAngle());
    SmartDashboard.getNumber("FRONT RIGHT ANGLE", frontRight.getAngle());
    SmartDashboard.getNumber("REAR LEFT ANGLE", rearLeft.getAngle());
    SmartDashboard.getNumber("REAR RIGHT ANGLE", rearRight.getAngle());
    
    SmartDashboard.getNumber("FRONT LEFT SPEED", frontLeft.getSpeed());
    SmartDashboard.getNumber("FRONT RIGHT SPEED", frontRight.getSpeed());
    SmartDashboard.getNumber("REAR LEFT SPEED", rearLeft.getSpeed());
    SmartDashboard.getNumber("REAR RIGHT SPEED", rearRight.getSpeed());

    SmartDashboard.getNumber("FRONT LEFT DISTANCE", frontLeft.getDistance());
    SmartDashboard.getNumber("FRONT RIGHT DISTANCE", frontRight.getDistance());
    SmartDashboard.getNumber("REAR LEFT DISTANCE", rearLeft.getDistance());
    SmartDashboard.getNumber("REAR RIGHT DISTANCE", rearRight.getDistance());
    
    SmartDashboard.getNumber("TARGET", Units.metersToInches(3));
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    OutputModuleInfo();
  }

  @Override
  public void simulationPeriodic() {

    }
}
