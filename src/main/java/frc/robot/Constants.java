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
    public static final int FRONT_LEFT_CANCODER_ID = 3;

    public static final int FRONT_RIGHT_FORWARD_ID = 4;
    public static final int FRONT_RIGHT_ROTATION_ID = 5;
    public static final int FRONT_RIGHT_CANCODER_ID = 6;

    public static final int REAR_LEFT_FORWARD_ID = 7;
    public static final int REAR_LEFT_ROTATION_ID = 8;
    public static final int REAR_LEFT_CANCODER_ID = 9;

    public static final int REAR_RIGHT_FORWARD_ID = 10;
    public static final int REAR_RIGHT_ROTATION_ID = 11;
    public static final int REAR_RIGHT_CANCODER_ID = 12;

    public static final int RIGHT_INTAKE_ID = 20;
    public static final int LEFT_INTAKE_ID = 21;

    public static final int FLYWHEEL_TOP_ID = 30;
    public static final int FLYWHEEL_BUTTOM_ID = 31;
  }

  public static class PhysicalConstants {
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double ROTATION_GEAR_RATIO = 150 / 7;

    public static final double MAX_VELOCITY_RPM = 217.5;
    public static final double MAX_VELOCITY_RPS = 3.625;

    public static final double SIDE_LENGTH = 38.0;
    public static final double SIDE_WIDTH = 22.0;
    public static final double SIDE_TO_CORNER = 21.9544984001;
    public static final double TRACK_WIDTH = Units.inchesToMeters(20);
    public static final double TRACK_LENGTH = Units.inchesToMeters(36);
    public static final double TRACK_RADIUS = 18.2242969686;


    public static final double LL_HEIGHT = 1;
    public static final double LL_CUBE_GOAL_HEIGHT = 2;
    public static final double LL_PICKUP_GOAL_HEIGHT = 4;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(TRACK_LENGTH/2, TRACK_WIDTH/2),
      new Translation2d(TRACK_LENGTH/2, -TRACK_WIDTH/2),
      new Translation2d(-TRACK_LENGTH/2, TRACK_WIDTH/2),
      new Translation2d(-TRACK_LENGTH/2, -TRACK_WIDTH/2)
    );

  }

  public static class PIDConstants {

    // -------------------------
    // DRIVE PID's
    // -------------------------

    public static final double FRONT_LEFT_FORWARD_PID0_P = 0.01;
    public static final double FRONT_LEFT_FORWARD_PID0_I = 0;
    public static final double FRONT_LEFT_FORWARD_PID0_D = 0;
    public static final double FRONT_LEFT_FORWARD_PID0_FF = 0.00011;

    public static final double FRONT_LEFT_ROTATION_PID0_P = 0.003405;
    public static final double FRONT_LEFT_ROTATION_PID0_I = 0;
    public static final double FRONT_LEFT_ROTATION_PID0_D = 0;
    public static final double FRONT_LEFT_ROTATION_PID0_FF = 0.00001;

    public static final double FRONT_RIGHT_FORWARD_PID0_P = 0.01;
    public static final double FRONT_RIGHT_FORWARD_PID0_I = 0;
    public static final double FRONT_RIGHT_FORWARD_PID0_D = 0;
    public static final double FRONT_RIGHT_FORWARD_PID0_FF = 0.00011;

    public static final double FRONT_RIGHT_ROTATION_PID0_P = 0.003405;
    public static final double FRONT_RIGHT_ROTATION_PID0_I = 0;
    public static final double FRONT_RIGHT_ROTATION_PID0_D = 0;
    public static final double FRONT_RIGHT_ROTATION_PID0_FF = 0.00002;

    public static final double REAR_LEFT_FORWARD_PID0_P = 0.01;
    public static final double REAR_LEFT_FORWARD_PID0_I = 0;
    public static final double REAR_LEFT_FORWARD_PID0_D = 0;
    public static final double REAR_LEFT_FORWARD_PID0_FF = 0.00011;

    public static final double REAR_LEFT_ROTATION_PID0_P = 0.003405;
    public static final double REAR_LEFT_ROTATION_PID0_I = 0;
    public static final double REAR_LEFT_ROTATION_PID0_D = 0;
    public static final double REAR_LEFT_ROTATION_PID0_FF = 0.00001;

    public static final double REAR_RIGHT_FORWARD_PID0_P = 0.01;
    public static final double REAR_RIGHT_FORWARD_PID0_I = 0;
    public static final double REAR_RIGHT_FORWARD_PID0_D = 0;
    public static final double REAR_RIGHT_FORWARD_PID0_FF = 0.00011;

    public static final double REAR_RIGHT_ROTATION_PID0_P = 0.003405;
    public static final double REAR_RIGHT_ROTATION_PID0_I = 0;
    public static final double REAR_RIGHT_ROTATION_PID0_D = 0;
    public static final double REAR_RIGHT_ROTATION_PID0_FF = 0.00001;

    // ------------------

    // ------------------
    // INTAKE PID's
    // ------------------

    public static final double INTAKE_PID0_P = 0;
    public static final double INTAKE_PID0_I = 0;
    public static final double INTAKE_PID0_D = 0;
    public static final double INTAKE_PID0_F = 0;
    public static final double INTAKE_ARB_FF = 0;

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
