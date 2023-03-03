package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  ///////////////////
  //   OP CONSTS   //
  ///////////////////

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /////////////////////
  //   CLAW CONSTS   //
  /////////////////////

  public static class ClawConsts {
    public static final int WRIST_MOTOR_PORT = 15;
    public static final int CLAW_FORWARD_CHANNEL = 0;
    public static final int CLAW_REVERSE_CHANNEL = 8;

    public static final double WRIST_SPEED = 0.75;

    public static final double ROTATE_90 = 95;
    public static final double ROTATION_T0_90_ENC = 95;
    public static final double ROTATION_TO_180_ENC = 190;
  }

  /////////////////////////
  //   ELEVATOR CONSTS   //
  /////////////////////////

  public static class ElevatorConsts {
    public static final int kDriverControllerPort = 0;
    public static final int ELEVATOR_ID = 13;
    public static final int TOP_LIMIT_SWITCH = 3;
    public static final int BOTTOM_LIMIT_SWITCH = 0;
    public static final int JOYSTICK = 0;
  }

  //////////////////////
  //   PIVOT CONSTS   //
  //////////////////////

  public static class PivotConsts {
    public static int PIVOT_MOTOR_PORT = 14;
    public static int PIVOT_LIMIT_PORT = 2;
  }

  /////////////////////////////
  //   LANDING GEAR CONSTS   //
  /////////////////////////////

  public static class LandingGearConsts{
    public static final int  LANDING_GEAR_PISTON_FORWARD_CHANNEL = 7;
    public static final int LANDING_GEAR_PISTON_REVERSE_CHANNEL = 15;
    public static final int LANDING_GEAR_MOTOR_PORT = 16;
  }

  /////////////////////
  //   AUTO CONSTS   //
  /////////////////////

  public static class AutoConsts{
    public static final double driveTranslationSpeed = 0.16;
    public static final double driveTranslationSlowSpeed = 0.07; 
    public static final double driveRotationSpeed = 0.3;

    public static final double balanceThreshhold = 13.5; //degree at which to stop balancing at 
    public static final double initialPitch = 12; //degree at which to stop for initial drive-up in auto 
    public static final double incrementalEncValue = 3; //encoder value traveled within each cycle 

  }

  ///////////////////////
  //   SWERVE CONSTS   //
  ///////////////////////

  public static class SwerveConsts{
    /* * * MEASUREMENTS * * */
    public static final double wheelDiameter = 4 * 2.5 / 100; // in meters
    public static final double gearRatio = 8.14 / 1;
    public static final double steerGearRatio = 150 / 7;
    public static final double trackWidth = 0.635;
    public static final double wheelBase = 0.635;
    public static final double voltage = 9.0;

    /* * * Swerve Drive Kinematics * * */
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
      // front left
      new Translation2d(wheelBase / 2, trackWidth / 2),
      // back left
      new Translation2d(-wheelBase / 2, trackWidth / 2),
      // back right
      new Translation2d(-wheelBase / 2, -trackWidth / 2),
      // front right
      new Translation2d(wheelBase / 2, -trackWidth / 2)
    );


    /* * * FRONT LEFT * * */
    public static final int FL_driveMotorPort = 1;
    public static final int FL_turningMotorPort = 5;
    public static final int FL_absoluteEncoderPort = 9;
    public static final double FL_offset = -Math.toRadians(3.155);

    /* * * BACK LEFT * * */
    public static final int BL_driveMotorPort = 2;
    public static final int BL_turningMotorPort = 6;
    public static final int BL_absoluteEncoderPort = 10;
    public static final double BL_offset = -Math.toRadians(3.118);

    /* * * BACK RIGHT * * */
    public static final int BR_driveMotorPort = 3;
    public static final int BR_turningMotorPort = 7;
    public static final int BR_absoluteEncoderPort = 11;
    public static final double BR_offset = -Math.toRadians(3.134);

    /* * * FRONT RIGHT * * */
    public static final int FR_driveMotorPort = 8;
    public static final int FR_turningMotorPort = 4;
    public static final int FR_absoluteEncoderPort = 12;
    public static final double FR_offset = -Math.toRadians(3.124);

    /* * * CONVERSIONS TO METERS * * */
    public static final double driveEncoderRotationConversion = gearRatio * Math.PI * wheelDiameter;
    public static final double driveEncoderSpeedConversion = driveEncoderRotationConversion / 60;

    /* * * CONVERSIONS TO RADIANS * * */
    public static final double turningEncoderRotationConversion = steerGearRatio * Math.PI; 
    public static final double turningEncoderSpeedConversion = turningEncoderRotationConversion / 60; 

    /* * * SPEEDS * * */
    public static final double maxSpeed_mps = 3.6576;
      // 0.10033 * Math.PI; // 13.5 feet per second = 4.1148 meters per second
    public static final double maxRotation = maxSpeed_mps / Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

    /* * * PID VALUES * * */
    public static final double kp_turning = 0.2;
    public static final double ki_turning = 0.0;
    public static final double kd_turning = 0.002;

    public static final double kp_driving = 0.002;
    public static final double ki_driving = 0.0;
    public static final double kd_driving = 0.0;
  }

  /* * * Landing Gear * * */
  public static final class LandingGear{
    public static final int deployingPistonFWDChannel = 0;
    public static final int deployingPistonREVChannel = 0;
    public static final int deployedWheelsPort = 0;
  }
}