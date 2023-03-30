package frc.robot.subsystems;

import java.sql.Time;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
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
      -52.1 + 180,
      true,
      false,
      false
    );

    public Module frontRight = new Module(
        Constants.MotorIDs.FRONT_RIGHT_FORWARD_ID, 
        Constants.MotorIDs.FRONT_RIGHT_ROTATION_ID,
        Constants.MotorIDs.FRONT_RIGHT_CANCODER_ID,
        66.44,
        false,
        false,
        false
    );

    public Module rearLeft = new Module(
        Constants.MotorIDs.REAR_LEFT_FORWARD_ID, 
        Constants.MotorIDs.REAR_LEFT_ROTATION_ID,
        Constants.MotorIDs.REAR_LEFT_CANCODER_ID,
        65 + 180,
        true,
        false,
        false
    );

    public Module rearRight = new Module(
        Constants.MotorIDs.REAR_RIGHT_FORWARD_ID, 
        Constants.MotorIDs.REAR_RIGHT_ROTATION_ID,
        Constants.MotorIDs.REAR_RIGHT_CANCODER_ID,
        -159,
        false,
        false,
        false
    );

    public XboxController driverJoystick = new XboxController(0);

    public Joystick operatorJoystick = new Joystick(1);

    private AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.PhysicalConstants.KINEMATICS, new Rotation2d(0), getSwerveModulePositions());

    private Pose2d pose = new Pose2d();

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(Constants.PhysicalConstants.KINEMATICS, new Rotation2d(0), getSwerveModulePositions(), new Pose2d());
    
    private Field2d field = new Field2d();

    private Relay relay = new Relay(0);

  public DriveSubsystem() {
    OutputModuleInfo();
    //gyro.calibrate();
    setup();
    //relay.set(Value.kReverse); 

    new Thread(() -> {
        try {
          Thread.sleep(2000);
          resetHeading();
        } catch (Exception e) {
        }
      }
    ).start();
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

  public void baselock(){
    stop();
    frontLeft.setAngle(60);
    frontRight.setAngle(-60);
    rearLeft.setAngle(-120);
    rearRight.setAngle(120);
  }

  public void setBrakeMode(boolean mode) {
    frontLeft.setBrakeMode(mode);
    frontRight.setBrakeMode(mode);
    rearLeft.setBrakeMode(mode);
    rearRight.setBrakeMode(mode);
  }

  /**
   * Resets the robot's encoders, gyro and odometry
   */
  public void reset(){
    frontLeft.reset();
    frontRight.reset();
    rearLeft.reset();
    rearRight.reset();
    resetHeading();
    odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), new Pose2d());
    poseEstimator.resetPosition(getRotation2d(), getSwerveModulePositions(), new Pose2d());
  }

  /**
   * Resets the pose to a desired position
   * 
   * @param pose desired pose of the robot
   */
  public void setPose(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
  }

  public void resetHeading() {
    gyro.zeroYaw();
  }

  /**
   * RETURNS THE HEADING OF THE ROBOT
   * IN ROTATION2D
   *
   * @return heading
  */
  public double getHeading() {
    return gyro.getYaw() * -1;
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  public void updateOdometry() {
    poseEstimator.update(getRotation2d(), getSwerveModulePositions());
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
    SwerveModulePosition frontleft = frontLeft.getModulePosition();
    SwerveModulePosition frontright = frontRight.getModulePosition();
    SwerveModulePosition rearleft = rearLeft.getModulePosition();
    SwerveModulePosition rearright = rearRight.getModulePosition();

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

    //SmartDashboard.putNumber("FRONT LEFT DISTANCE", frontLeft.getDistance());
    //SmartDashboard.putNumber("FRONT RIGHT DISTANCE", frontRight.getDistance());
    //SmartDashboard.putNumber("REAR LEFT DISTANCE", rearLeft.getDistance());
    //SmartDashboard.putNumber("REAR RIGHT DISTANCE", rearRight.getDistance());

    SmartDashboard.putNumber("ROTATION", getRotation2d().getDegrees());
    SmartDashboard.putNumber("POSE ANGLE", getPose().getRotation().getDegrees());
  
    //SmartDashboard.putNumber("PITCH", getPitch());
    SmartDashboard.putNumber("GET ROLL", getRoll());

    SmartDashboard.putData(field);
  }

  /**
   * Outputs robot information,
   * to be used for autonomous purposes
   * 
   * @param states Desires states of the swerve modules
   * 
   * @return Makes the robot move
   * @return Output Information aboutb the robot on shuffleboard
   */
  public void OutputModuleInfo(SwerveModuleState[] states) {
    //SmartDashboard.putNumber("FRONT LEFT ANGLE", frontLeft.getAngle());
    //SmartDashboard.putNumber("FRONT RIGHT ANGLE", frontRight.getAngle());
    //SmartDashboard.putNumber("REAR LEFT ANGLE", rearLeft.getAngle());
    //SmartDashboard.putNumber("REAR RIGHT ANGLE", rearRight.getAngle());

    //SmartDashboard.putNumber("FRONT LEFT ABSOLUTE", frontLeft.getAbsoluteAngle());
    //SmartDashboard.putNumber("FRONT RIGHT ABSOLUTE", frontRight.getAbsoluteAngle());
    //SmartDashboard.putNumber("REAR LEFT ABSOLUTE", rearLeft.getAbsoluteAngle());
    //SmartDashboard.putNumber("REAR RIGHT ABSOLUTE", rearRight.getAbsoluteAngle());
    
    //SmartDashboard.putNumber("FRONT LEFT SPEED", frontLeft.getSpeed());
    //SmartDashboard.putNumber("FRONT RIGHT SPEED", frontRight.getSpeed());
    //SmartDashboard.putNumber("REAR LEFT SPEED", rearLeft.getSpeed());
    //SmartDashboard.putNumber("REAR RIGHT SPEED", rearRight.getSpeed());

    //SmartDashboard.putNumber("FRONT LEFT DISTANCE", frontLeft.getDistance());
    //SmartDashboard.putNumber("FRONT RIGHT DISTANCE", frontRight.getDistance());
    //SmartDashboard.putNumber("REAR LEFT DISTANCE", rearLeft.getDistance());
    //SmartDashboard.putNumber("REAR RIGHT DISTANCE", rearRight.getDistance());
  
    SmartDashboard.putNumber("ANGLE", getRotation2d().getDegrees());

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.PhysicalConstants.MAX_METERS_PER_SECOND);

    frontLeft.setState(states[0], false);
    frontRight.setState(states[1], false);
    rearLeft.setState(states[2], false);
    rearRight.setState(states[3], false);
  }

  public void setStates(SwerveModuleState[] state) {
    frontLeft.setState(state[0], true); // 2
    frontRight.setState(state[1], true); // 3
    rearLeft.setState(state[2], true); // 0
    rearRight.setState(state[3], true); // 1

    //SmartDashboard.putNumber("FRONT LEFT TARGET", state[0].angle.getDegrees());
    //SmartDashboard.putNumber("FRONT RIGHT TARGET", state[1].angle.getDegrees());
    //SmartDashboard.putNumber("REAR LEFT TARGET", state[2].angle.getDegrees());
    //SmartDashboard.putNumber("REAR RIGHT TARGET", state[3].angle.getDegrees());
  }

  public void OutputChassisSpeeds(ChassisSpeeds speeds) {
    OutputModuleInfo(Constants.PhysicalConstants.KINEMATICS.toSwerveModuleStates(speeds));
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(), getSwerveModulePositions());
    updateOdometry();
    OutputModuleInfo();
    field.setRobotPose(odometry.getPoseMeters());
  }

  @Override
  public void simulationPeriodic() {

    }
}