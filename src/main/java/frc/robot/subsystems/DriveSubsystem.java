package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private DriveSubsystem instance;

    public Module frontLeft = new Module(
      Constants.MotorIDs.FRONT_LEFT_FORWARD_ID,
      Constants.MotorIDs.FRONT_LEFT_ROTATION_ID,
      Constants.MotorIDs.FRONT_LEFT_CANCODER_ID,
      -52.1,
      true,
      false
    );

    public Module frontRight = new Module(
        Constants.MotorIDs.FRONT_RIGHT_FORWARD_ID, 
        Constants.MotorIDs.FRONT_RIGHT_ROTATION_ID,
        Constants.MotorIDs.FRONT_RIGHT_CANCODER_ID,
        66.44,
        true,
        false
    );

    public Module rearLeft = new Module(
        Constants.MotorIDs.REAR_LEFT_FORWARD_ID, 
        Constants.MotorIDs.REAR_LEFT_ROTATION_ID,
        Constants.MotorIDs.REAR_LEFT_CANCODER_ID,
        65,
        true,
        false
    );

    public Module rearRight = new Module(
        Constants.MotorIDs.REAR_RIGHT_FORWARD_ID, 
        Constants.MotorIDs.REAR_RIGHT_ROTATION_ID,
        Constants.MotorIDs.REAR_RIGHT_CANCODER_ID,
        -159,
        true,
        false
    );

    public Joystick driverJoystick = new Joystick(0);

    private AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.PhysicalConstants.KINEMATICS, new Rotation2d(0), getSwerveModulePositions());

    private Pose2d pose = new Pose2d();

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(Constants.PhysicalConstants.KINEMATICS, new Rotation2d(0), getSwerveModulePositions(), new Pose2d());
    
    private Field2d field = new Field2d();

    private Relay relay = new Relay(0);

  public DriveSubsystem() {
    OutputModuleInfo();
    gyro.calibrate();
    setup();
    relay.set(Value.kReverse);    
  }
 
  public void getInstance(){
    if(instance == null){
      instance = new DriveSubsystem();
    }
  }

  public void setup(){
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

  /**
   * Stops the drive motors in the robots
   */
  public void stop() {
    frontLeft.stopMotors();
    frontRight.stopMotors();
    rearLeft.stopMotors();
    rearRight.stopMotors();
  }

  /**
   * Resets the robot's encoders, gyro and odometry
   */
  public void reset(){
    frontLeft.reset();
    frontRight.reset();
    rearLeft.reset();
    rearRight.reset();
    resetGyro();
    odometry.resetPosition(getHeading(), getSwerveModulePositions(), new Pose2d());
    poseEstimator.resetPosition(getHeading(), getSwerveModulePositions(), new Pose2d());
  }

  public void setPose(Pose2d pose) {
    odometry.resetPosition(getHeading(), getSwerveModulePositions(), pose);
  }

  public void resetGyro() {
    gyro.reset();
  }

  /**
   * RETURNS THE HEADING OF THE ROBOT
   * IN ROTATION2D
   *
   * @return heading
  */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public void updateOdometry() {
    odometry.update(getHeading(), getSwerveModulePositions());
  }

  /**
   * RETURNS THE POSE OF THE ROBOT
   * 
   * @return pose
  */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition frontleft = new SwerveModulePosition(frontLeft.getDistance(), frontLeft.getRotation());
    SwerveModulePosition frontright = new SwerveModulePosition(frontRight.getDistance(), frontRight.getRotation());
    SwerveModulePosition rearleft = new SwerveModulePosition(rearLeft.getDistance(), rearLeft.getRotation());
    SwerveModulePosition rearright = new SwerveModulePosition(rearRight.getDistance(), rearRight.getRotation());

    SwerveModulePosition swerveModulePosition[] = {frontleft, frontright, rearleft, rearright};

    return swerveModulePosition;
  }

  public void OutputModuleInfo() {
    SmartDashboard.putNumber("FRONT LEFT ANGLE", frontLeft.getAngleCancoder());
    SmartDashboard.putNumber("FRONT RIGHT ANGLE", frontRight.getAngleCancoder());
    SmartDashboard.putNumber("REAR LEFT ANGLE", rearLeft.getAngleCancoder());
    SmartDashboard.putNumber("REAR RIGHT ANGLE", rearRight.getAngleCancoder());
    
    SmartDashboard.putNumber("FRONT LEFT ABSOLUTE", frontLeft.getAbsoluteAngle());
    SmartDashboard.putNumber("FRONT RIGHT ABSOLUTE", frontRight.getAbsoluteAngle());
    SmartDashboard.putNumber("REAR LEFT ABSOLUTE", rearLeft.getAbsoluteAngle());
    SmartDashboard.putNumber("REAR RIGHT ABSOLUTE", rearRight.getAbsoluteAngle());

    SmartDashboard.putNumber("FRONT LEFT SPEED", frontLeft.getSpeed());
    SmartDashboard.putNumber("FRONT RIGHT SPEED", frontRight.getSpeed());
    SmartDashboard.putNumber("REAR LEFT SPEED", rearLeft.getSpeed());
    SmartDashboard.putNumber("REAR RIGHT SPEED", rearRight.getSpeed());

    SmartDashboard.putNumber("FRONT LEFT DISTANCE", frontLeft.getDistance());
    SmartDashboard.putNumber("FRONT RIGHT DISTANCE", frontRight.getDistance());
    SmartDashboard.putNumber("REAR LEFT DISTANCE", rearLeft.getDistance());
    SmartDashboard.putNumber("REAR RIGHT DISTANCE", rearRight.getDistance());

    SmartDashboard.putNumber("ANGLE", gyro.getAngle());
    SmartDashboard.putNumber("POSE ANGLE", getPose().getRotation().getDegrees());
  
    SmartDashboard.putData(field);
  }

  /**
   * Outputs robot information
   * ,to be used for autonomous purposes
   * 
   * @param states Desires states of the swerve modules
   * 
   * @return Makes the robot move while outputing robot information
   */
  public void OutputModuleInfo(SwerveModuleState[] states) {
    SmartDashboard.putNumber("FRONT LEFT ANGLE", frontLeft.getAngle());
    SmartDashboard.putNumber("FRONT RIGHT ANGLE", frontRight.getAngle());
    SmartDashboard.putNumber("REAR LEFT ANGLE", rearLeft.getAngle());
    SmartDashboard.putNumber("REAR RIGHT ANGLE", rearRight.getAngle());

    SmartDashboard.putNumber("FRONT LEFT ABSOLUTE", frontLeft.getAbsoluteAngle());
    SmartDashboard.putNumber("FRONT RIGHT ABSOLUTE", frontRight.getAbsoluteAngle());
    SmartDashboard.putNumber("REAR LEFT ABSOLUTE", rearLeft.getAbsoluteAngle());
    SmartDashboard.putNumber("REAR RIGHT ABSOLUTE", rearRight.getAbsoluteAngle());
    
    SmartDashboard.putNumber("FRONT LEFT SPEED", frontLeft.getSpeed());
    SmartDashboard.putNumber("FRONT RIGHT SPEED", frontRight.getSpeed());
    SmartDashboard.putNumber("REAR LEFT SPEED", rearLeft.getSpeed());
    SmartDashboard.putNumber("REAR RIGHT SPEED", rearRight.getSpeed());

    SmartDashboard.putNumber("FRONT LEFT DISTANCE", frontLeft.getDistance());
    SmartDashboard.putNumber("FRONT RIGHT DISTANCE", frontRight.getDistance());
    SmartDashboard.putNumber("REAR LEFT DISTANCE", rearLeft.getDistance());
    SmartDashboard.putNumber("REAR RIGHT DISTANCE", rearRight.getDistance());
  
    SmartDashboard.putNumber("ANGLE", gyro.getAngle());

    //SwerveDriveKinematics.desaturateWheelSpeeds(states, null, 12, 10, 8);

    frontLeft.setState(states[0], false);
    frontRight.setState(states[1], false);
    rearLeft.setState(states[2], false);
    rearRight.setState(states[3], false);

    SmartDashboard.putNumber("FRONT LEFT DISTANCE", states[0].angle.getDegrees());
    SmartDashboard.putNumber("FRONT RIGHT DISTANCE", states[1].angle.getDegrees());
    SmartDashboard.putNumber("REAR LEFT DISTANCE", states[2].angle.getDegrees());
    SmartDashboard.putNumber("REAR RIGHT DISTANCE", states[3].angle.getDegrees());
  }

  public void setStates(SwerveModuleState[] state) {
    frontLeft.setState(state[2], true); // 2
    frontRight.setState(state[3], true); // 3
    rearLeft.setState(state[0], true); // 0
    rearRight.setState(state[1], true); // 1

    SmartDashboard.putNumber("FRONT LEFT TARGET", state[2].angle.getDegrees());
    SmartDashboard.putNumber("FRONT RIGHT TARGET", state[3].angle.getDegrees());
    SmartDashboard.putNumber("REAR LEFT TARGET", state[0].angle.getDegrees());
    SmartDashboard.putNumber("REAR RIGHT TARGET", state[1].angle.getDegrees());
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), getSwerveModulePositions());
    updateOdometry();
    OutputModuleInfo();
    field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {

    }
}