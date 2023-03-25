package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class DriverControlConsts {
    public static final int XBOX_CONTROLLER_PORT = 0;
    public static final int JOYSTICK_PORT = 1;
  }

  public static class SwerveConsts{
    /* * * MEASUREMENTS * * */
    public static final double WHEEL_DIAMETER = 4 * 2.5 / 100;
    public static final double TRACK_WIDTH = 0.635;
    public static final double WHEEL_BASE = 0.635;

    public static final double GEAR_RATIO = 8.14 / 1;
    public static final double STEER_GEAR_RATIO = 150 / 7;

    public static final double VOLTAGE = 7.2;

    /* * * SWERVE DRIVE KINEMATICS * * */
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      // front left
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // back left
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // back right
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      // front right
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    /* * * FRONT LEFT * * */
    public static final int FL_DRIVE_PORT = 1;
    public static final int FL_TURN_PORT = 5;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 9;
    public static final double FL_OFFSET = -Math.toRadians(3.174);

    /* * * BACK LEFT * * */
    public static final int BL_DRIVE_PORT = 2;
    public static final int BL_TURN_PORT = 6;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 10;
    public static final double BL_OFFSET = -Math.toRadians(3.120);

    /* * * BACK RIGHT * * */
    public static final int BR_DRIVE_PORT = 3;
    public static final int BR_TURN_PORT = 7;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 11;
    public static final double BR_OFFSET = -Math.toRadians(3.106);

    /* * * FRONT RIGHT * * */
    public static final int FR_DRIVE_PORT = 8;
    public static final int FR_TURN_PORT = 4;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 12;
    public static final double FR_OFFSET = -Math.toRadians(3.042);

    /* * * CONVERSIONS FOR ENCODERS * * */
    public static final double DRIVE_ENCODER_ROTATION_CONVERSION = GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double DRIVE_ENCODER_SPEED_CONVERSION = DRIVE_ENCODER_ROTATION_CONVERSION / 60;

    public static final double TURNING_ENCODER_ROTATION_CONVERSION = STEER_GEAR_RATIO * Math.PI;
    public static final double TURNING_ENCODER_SPEED_CONVERSION = TURNING_ENCODER_ROTATION_CONVERSION / 60;
      // 5676 / 60 * 341.88 * Math.PI

    /* * * MAX SPEEDS * * */
    public static final double MAX_SPEED = 3.6576;
      // 5676.0 / 60.0 * ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)) *
      // 0.10033 * Math.PI; // 13.5 feet per second = 4.1148 meters per second
    public static final double MAX_ROTATION = MAX_SPEED / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

    /* * * PID VALUES * * */
    public static final double KP_TURNING = 0.5;
    public static final double KI_TURNING = 0.0;
    public static final double KD_TURNING = 0.0025;
  }

  public static class LandingGearConsts{
    public static final int  LANDING_GEAR_PISTON_FORWARD_CHANNEL = 7;
    public static final int LANDING_GEAR_PISTON_REVERSE_CHANNEL = 15;
    public static final int LANDING_GEAR_MOTOR_PORT = 16;
  }

  public static class ElevatorConsts {
    public static final int ELEVATOR_MOTOR_PORT = 13;
    public static final int TOP_LIMIT_PORT = 3;
    public static final int BOTTOM_LIMIT_PORT = 0;
  }

  public static class PivotConsts {
    public static final int PIVOT_MOTOR_PORT = 14;
    public static final int PIVOT_LIMIT_PORT = 2;
  }

  public static class ClawConsts {
    public static final int WRIST_MOTOR_PORT = 15;
    public static final int CLAW_FORWARD_CHANNEL = 0;
    public static final int CLAW_REVERSE_CHANNEL = 8;

    public static final double WRIST_SPEED = 0.8;

    public static final double ROTATE_90 = 95;
    public static final double ROTATION_T0_90_ENC = 95;
    public static final double ROTATION_TO_180_ENC = 190;
  }

  public static class AutoConsts {
    public static final double DRIVE_TRANSLATION_SPEED = 0.2;
    public static final double DRIVE_ROTATION_SPEED = 0.25;
  }

}
