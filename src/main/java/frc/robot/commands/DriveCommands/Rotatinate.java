package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class Rotatinate extends CommandBase {
  private SwerveSubsystem swerve;
  private DoubleSupplier x, y;
  private PIDController pid;
  private double desiredYaw;

  public Rotatinate(SwerveSubsystem subs, DoubleSupplier x, DoubleSupplier y) {
    swerve = subs;
    this.x = x;
    this.y = y;

    pid = new PIDController(0.002, 0, 0);
    pid.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(subs);
  }

  @Override
  public void initialize() {
    desiredYaw = Math.atan2(x.getAsDouble(), y.getAsDouble());
  }

  @Override
  public void execute() {

    double yaw = swerve.getRobotRotation().getDegrees();

    double rotate = pid.calculate(yaw, desiredYaw);

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, rotate);
    
    SwerveModuleState[] states = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    swerve.setModuleStates(states);

    SmartDashboard.putNumber("Desired Yaw", desiredYaw);
    SmartDashboard.putNumber("Current Yaw", swerve.getRobotRotation().getDegrees());
    SmartDashboard.putNumber("Calculation", rotate);

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return Math.abs(desiredYaw-swerve.getYawAngle()) <= 10;
  }
}
