package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class Rotate180 extends CommandBase {
  private SwerveSubsystem swerve;
  private DoubleSupplier x, y;
  private PIDController pid;
  private double desiredYaw;

  public Rotate180(SwerveSubsystem subs, DoubleSupplier x, DoubleSupplier y) {
    swerve = subs;
    this.x = x;
    this.y = y;

    addRequirements(subs);
  }

  @Override
  public void initialize() {
    desiredYaw = 180;
    desiredYaw = desiredYaw<=-180 ? desiredYaw+360 : desiredYaw;
  }

  @Override
  public void execute() {

    double yaw = swerve.getYawAngle();

    ChassisSpeeds chassisSpeeds = yaw < desiredYaw ? new ChassisSpeeds(0, 0, 0.5) : new ChassisSpeeds(0, 0, -0.5);
    
    SwerveModuleState[] states = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    swerve.setModuleStates(states);

    SmartDashboard.putNumber("Desired Yaw", desiredYaw);
    SmartDashboard.putNumber("Current Yaw", swerve.getRobotRotation().getDegrees());

  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.clearPersistent("Desired Yaw");
    SmartDashboard.clearPersistent("Current Yaw");
  }

  @Override
  public boolean isFinished() {
    return (desiredYaw-5 <= swerve.getYawAngle()) && (swerve.getYawAngle() <= 5) ;
  }
}
