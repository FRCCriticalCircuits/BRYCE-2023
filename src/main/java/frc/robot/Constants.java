package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_GAMEPAD_ID = 0;
    public static final int OPERATOR_GAMEPAD_ID = 1;

    // -----------------------------
    //  OPERATORS THRESHOLDS
    // -----------------------------

    public static final double DRIVER_X1_THRESHOLD = 0.01;
    public static final double DRIVER_X2_THRESHOLD = 0.01;
    public static final double DRIVER_Y1_THRESHOLD = 0.01;
    public static final double DRIVER_Y2_THRESHOLD = 0.01;

    public static final double OPERATOR_X1_THEESHOLD = 0.01;
    public static final double OPERATOR_X2_THRESHOLD = 0.01;
    public static final double OPERATOR_Y1_THRESHOLD = 0.01;
    public static final double OPERATOR_Y2_THRESHOLD = 0.01;
  }

  public static class MotorIDs {
    public static final int FRONT_LEFT_FORWARD_ID = 1;
    public static final int FRONT_LEFT_ROTATION_ID = 2;
    
    public static final int FRONT_RIGHT_FORWARD_ID = 4;
    public static final int FRONT_RIGHT_ROTATION_ID = 5;

    public static final int REAR_LEFT_FORWARD_ID = 7;
    public static final int REAR_LEFT_ROTATION_ID = 8;

    public static final int REAR_RIGHT_FORWARD_ID = 10;
    public static final int REAR_RIGHT_ROTATION_ID = 11;
  }

  public static class PhysicalConstants {
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double ROTATION_GEAR_RATIO = 150 / 7;

    public static final double MAX_VELOCITY_RPM = 217.5;
    public static final double MAX_VELOCITY_RPS = 3.625;

    public static final double SIDE_LENGTH = 38.0;
    public static final double SIDE_WIDTH = 22.0;
    public static final double SIDE_TO_CORNER = 21.9544984001;

    public static final double LL_HEIGHT = 1;
    public static final double LL_CUBE_GOAL_HEIGHT = 2;
    public static final double LL_PICKUP_GOAL_HEIGHT = 4;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(Units.inchesToMeters(19), Units.inchesToMeters(11)),
      new Translation2d(-Units.inchesToMeters(19), Units.inchesToMeters(11)),
      new Translation2d(Units.inchesToMeters(19), -Units.inchesToMeters(11)),
      new Translation2d(-Units.inchesToMeters(19), -Units.inchesToMeters(11))
    );

  }

  public static class PIDConstants {

    // -------------------------
    // DRIVE PID's
    // -------------------------

    public static final double FRONT_LEFT_FORWARD_PID0_P = 0.001;
    public static final double FRONT_LEFT_FORWARD_PID0_I = 0;
    public static final double FRONT_LEFT_FORWARD_PID0_D = 0;
    public static final double FRONT_LEFT_FORWARD_PID0_FF = 0.00011;

    public static final double FRONT_LEFT_ROTATION_PID0_P = 0.003405;
    public static final double FRONT_LEFT_ROTATION_PID0_I = 0;
    public static final double FRONT_LEFT_ROTATION_PID0_D = 0;
    public static final double FRONT_LEFT_ROTATION_PID0_FF = 0.00001;

    public static final double FRONT_RIGHT_FORWARD_PID0_P = 0.001;
    public static final double FRONT_RIGHT_FORWARD_PID0_I = 0;
    public static final double FRONT_RIGHT_FORWARD_PID0_D = 0;
    public static final double FRONT_RIGHT_FORWARD_PID0_FF = 0.00011;

    public static final double FRONT_RIGHT_ROTATION_PID0_P = 0.003405;
    public static final double FRONT_RIGHT_ROTATION_PID0_I = 0;
    public static final double FRONT_RIGHT_ROTATION_PID0_D = 0;
    public static final double FRONT_RIGHT_ROTATION_PID0_FF = 0.00001;

    public static final double REAR_LEFT_FORWARD_PID0_P = 0.001;
    public static final double REAR_LEFT_FORWARD_PID0_I = 0;
    public static final double REAR_LEFT_FORWARD_PID0_D = 0;
    public static final double REAR_LEFT_FORWARD_PID0_FF = 0.00011;

    public static final double REAR_LEFT_ROTATION_PID0_P = 0.003405;
    public static final double REAR_LEFT_ROTATION_PID0_I = 0;
    public static final double REAR_LEFT_ROTATION_PID0_D = 0;
    public static final double REAR_LEFT_ROTATION_PID0_FF = 0.00001;

    public static final double REAR_RIGHT_FORWARD_PID0_P = 0.001;
    public static final double REAR_RIGHT_FORWARD_PID0_I = 0;
    public static final double REAR_RIGHT_FORWARD_PID0_D = 0;
    public static final double REAR_RIGHT_FORWARD_PID0_FF = 0.00011;

    public static final double REAR_RIGHT_ROTATION_PID0_P = 0.003405;
    public static final double REAR_RIGHT_ROTATION_PID0_I = 0;
    public static final double REAR_RIGHT_ROTATION_PID0_D = 0;
    public static final double REAR_RIGHT_ROTATION_PID0_FF = 0.00001;

    // ------------------

    // ------------------------
    //  OTHER MOTOR PID's
    // ------------------------

    public static final double WINDOW_PID0_P = 1;
    public static final double WINDOW_PID0_I = 0;
    public static final double WINDOW_PID0_D = 0;
    public static final double WINDOW_PID0_F = 0;

    public static final double FLYWHEEL_PID0_P = 1;
    public static final double FLYWHEEL_PID0_I = 0;
    public static final double FLYWHEEL_PID0_D = 0;
    public static final double FLYWHEEL_PID0_F = 0;

    public static final double ARM_PID0_P = 1;
    public static final double ARM_PID0_I = 0;
    public static final double ARM_PID0_D = 0;
    public static final double ARM_PID0_F = 0;

    // ------------------------

    public static final double CRITICAL_X_PID0_P = 1;
    public static final double CRITICAL_X_PID0_I = 0;
    public static final double CRITICAL_X_PID0_D = 0;

    public static final double CRITICAL_Y_PID0_P = 1;
    public static final double CRITICAL_Y_PID0_I = 0;
    public static final double CRITICAL_Y_PID0_D = 0;

    public static final double CRITICAL_THETA_PID0_P = 1;
    public static final double CRITICAL_THETA_PID0_I = 0;
    public static final double CRITICAL_THETA_PID0_D = 0;
  }
}
