package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.Constants.SwerveConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class Rotate180 extends CommandBase {
  private SwerveSubsystem swerve;
  private DoubleSupplier xSupplier, ySupplier;

  public Rotate180(SwerveSubsystem s, DoubleSupplier x, DoubleSupplier y) {
    swerve = s;
    xSupplier = x;
    ySupplier = y;
    addRequirements(s);
  }

@Override
  public void initialize() {
    swerve.resetNavx();
  }

  @Override
  public void execute() {
    // Get values from joysticks
    double xSpeed = xSupplier.getAsDouble();
    double ySpeed = ySupplier.getAsDouble();

    // Deadzone
    xSpeed = deadzone(xSpeed);
    ySpeed = deadzone(ySpeed);

    // Smoother acceleration
    xSpeed = modifyAxis(xSpeed); //xLimiter.calculate(xSpeed) * SwerveConsts.maxSpeed_mps;
    ySpeed = modifyAxis(ySpeed); //yLimiter.calculate(ySpeed) * SwerveConsts.maxSpeed_mps;

    // Chassis Speeds
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0.25);

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

    // Set each module state to wheels
    swerve.setModuleStates(moduleStates);
    
    SmartDashboard.putBoolean("ROTATING 180", true);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("ROTATING 180", false);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(swerve.getYaw() - 180) < 5;
  }

  public double deadzone(double num) {
    return Math.abs(num) > 0.01 ? num : 0;
  }

  private static double modifyAxis(double num) {
    // Square the axis
    num = Math.copySign(num * num, num);

    return num;
  }
}
