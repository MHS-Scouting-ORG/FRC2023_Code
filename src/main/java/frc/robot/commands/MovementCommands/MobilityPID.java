package frc.robot.commands.MovementCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class MobilityPID extends CommandBase {
  private final SwerveSubsystem swerve;
  private final PIDController drivePID;
  private final PIDController turningPID;

  private double desiredEnc;
  private double desiredAngle;

  private double currentTurningError;
  private double previousTurningError = 0;

  private double currentDrivingError;
  private double previousDrivingError = 0;

  public MobilityPID(SwerveSubsystem newSwerve, double newDesiredEnc, double newDesiredAngle) {
    swerve = newSwerve;
    desiredEnc = newDesiredEnc;
    desiredAngle = newDesiredAngle;
    drivePID = new PIDController(0.01, 0, 0);
    turningPID = new PIDController(0.01, 0, 0);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.resetEnc();
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());

    
    // TURNING PID
    double turningSpeed = turningPID.calculate(swerve.getYaw(), desiredAngle);

    if (turningSpeed > 0.5) {
      turningSpeed = 0.5;
    } else if (turningSpeed < -0.5) {
      turningSpeed = -0.5;
    }

    currentTurningError = desiredAngle - swerve.getYaw();

    if (currentTurningError > 0 && previousTurningError < 0) {
      turningPID.reset();
    } else if (currentTurningError < 0 && previousTurningError > 0) {
      turningPID.reset();
    }

    previousTurningError = currentTurningError;
    SmartDashboard.putNumber("turningSpeed", turningSpeed);


    // DRIVING PID
    double drivingSpeed = drivePID.calculate(swerve.getDriveEnc(), desiredEnc);

    if (drivingSpeed > 0.5) {
      drivingSpeed = 0.5;
    } else if (drivingSpeed < -0.5) {
      drivingSpeed = -0.5;
    }

    currentDrivingError = desiredEnc - swerve.getDriveEnc();

    if (currentDrivingError > 0 && previousDrivingError < 0) {
      drivePID.reset();
    } else if (currentDrivingError < 0 && previousDrivingError > 0) {
      drivePID.reset();
    }

    previousDrivingError = currentDrivingError;
    SmartDashboard.putNumber("drivingSpeed", drivingSpeed);

    // Setting chassis speeds
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(drivingSpeed, 0, turningSpeed, swerve.getRotation2d());

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    // Set each module state to wheels
    swerve.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(currentDrivingError) < 2 && Math.abs(currentTurningError) < 2;
  }
}
