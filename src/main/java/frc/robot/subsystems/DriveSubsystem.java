package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
    private DriveSubsystem instance;

    public Module frontLeft = new Module(
      Constants.MotorIDs.FRONT_LEFT_FORWARD_ID,
      Constants.MotorIDs.FRONT_LEFT_ROTATION_ID,
      Constants.MotorIDs.FRONT_LEFT_CANCODER_ID,
      0
    );

    public Module frontRight = new Module(
        Constants.MotorIDs.FRONT_RIGHT_FORWARD_ID, 
        Constants.MotorIDs.FRONT_RIGHT_ROTATION_ID,
        Constants.MotorIDs.FRONT_RIGHT_CANCODER_ID,
        0
    );

    public Module rearLeft = new Module(
        Constants.MotorIDs.REAR_LEFT_FORWARD_ID, 
        Constants.MotorIDs.REAR_LEFT_ROTATION_ID,
        Constants.MotorIDs.REAR_LEFT_CANCODER_ID,
        0
    );

    public Module rearRight = new Module(
        Constants.MotorIDs.REAR_RIGHT_FORWARD_ID, 
        Constants.MotorIDs.REAR_RIGHT_ROTATION_ID,
        Constants.MotorIDs.REAR_RIGHT_CANCODER_ID,
        0
    );

    public Joystick driverJoystick = new Joystick(0);

    private AnalogGyro gyro = new AnalogGyro(0);

    private SwerveModulePosition[] modulePositions = {
      new SwerveModulePosition(frontLeft.getDistance(), frontLeft.getRotation()),
      new SwerveModulePosition(frontRight.getDistance(), frontRight.getRotation()),
      new SwerveModulePosition(rearLeft.getDistance(), rearLeft.getRotation()),
      new SwerveModulePosition(rearRight.getDistance(), rearRight.getRotation())
    };

    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(Constants.PhysicalConstants.KINEMATICS, getHeading(), modulePositions);

    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(Constants.PhysicalConstants.KINEMATICS, getHeading(), modulePositions, new Pose2d());
    
  public DriveSubsystem() {
    OutputModuleInfo();
    setup();
  }

  /*
  public void getInstance(){
    if(instance == null){
      instance = new DriveSubsystem();
    }
  }
 */

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

  public void stop() {
    frontLeft.stopMotors();
    frontRight.stopMotors();
    rearLeft.stopMotors();
    rearRight.stopMotors();
  }

  public void reset(){
    frontLeft.resetEncoderForward();
    frontRight.resetEncoderForward();
    rearLeft.resetEncoderForward();
    rearRight.resetEncoderForward();
    odometry.resetPosition(getHeading(), modulePositions, new Pose2d());
    //gyro.reset();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  /*
    RETURNS THE POSE OF THE ROBOT
    
    @return pose
  */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    
    return null;
  }

  public void OutputModuleInfo() {
    SmartDashboard.putNumber("FRONT LEFT ANGLE", frontLeft.getAngle());
    SmartDashboard.putNumber("FRONT RIGHT ANGLE", frontRight.getAngle());
    SmartDashboard.putNumber("REAR LEFT ANGLE", rearLeft.getAngle());
    SmartDashboard.putNumber("REAR RIGHT ANGLE", rearRight.getAngle());
    
    SmartDashboard.putNumber("FRONT LEFT SPEED", frontLeft.getSpeed());
    SmartDashboard.putNumber("FRONT RIGHT SPEED", frontRight.getSpeed());
    SmartDashboard.putNumber("REAR LEFT SPEED", rearLeft.getSpeed());
    SmartDashboard.putNumber("REAR RIGHT SPEED", rearRight.getSpeed());

    SmartDashboard.putNumber("FRONT LEFT DISTANCE", frontLeft.getDistance());
    SmartDashboard.putNumber("FRONT RIGHT DISTANCE", frontRight.getDistance());
    SmartDashboard.putNumber("REAR LEFT DISTANCE", rearLeft.getDistance());
    SmartDashboard.putNumber("REAR RIGHT DISTANCE", rearRight.getDistance());
  }

  public void OutputModuleInfo(SwerveModuleState[] states) {
    SmartDashboard.putNumber("FRONT LEFT ANGLE", frontLeft.getAngle());
    SmartDashboard.putNumber("FRONT RIGHT ANGLE", frontRight.getAngle());
    SmartDashboard.putNumber("REAR LEFT ANGLE", rearLeft.getAngle());
    SmartDashboard.putNumber("REAR RIGHT ANGLE", rearRight.getAngle());
    
    SmartDashboard.putNumber("FRONT LEFT SPEED", frontLeft.getSpeed());
    SmartDashboard.putNumber("FRONT RIGHT SPEED", frontRight.getSpeed());
    SmartDashboard.putNumber("REAR LEFT SPEED", rearLeft.getSpeed());
    SmartDashboard.putNumber("REAR RIGHT SPEED", rearRight.getSpeed());

    SmartDashboard.putNumber("FRONT LEFT DISTANCE", frontLeft.getDistance());
    SmartDashboard.putNumber("FRONT RIGHT DISTANCE", frontRight.getDistance());
    SmartDashboard.putNumber("REAR LEFT DISTANCE", rearLeft.getDistance());
    SmartDashboard.putNumber("REAR RIGHT DISTANCE", rearRight.getDistance());
  
    frontLeft.setState(states[0]);
    frontRight.setState(states[1]);
    rearLeft.setState(states[2]);
    rearRight.setState(states[3]);
  }

  public void setStates(SwerveModuleState[] state) {
    frontLeft.setState(state[0]);
    frontRight.setState(state[1]);
    rearLeft.setState(state[2]);
    rearRight.setState(state[3]);
  }

  @Override
  public void periodic() {
    poseEstimator.update(getHeading(), modulePositions);
    odometry.update(getHeading(), modulePositions);
    OutputModuleInfo();
  }

  @Override
  public void simulationPeriodic() {

    }
}