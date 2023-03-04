package frc.robot.commands.MovementCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateLeftCommand extends CommandBase {
  private final SwerveSubsystem swerve; 
  private double desiredAngle;

  public RotateLeftCommand(SwerveSubsystem newSwerve, double angle) {
    swerve = newSwerve;
    desiredAngle = angle;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    swerve.rotateLeft(AutoConsts.DRIVE_ROTATION_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return desiredAngle-2 < swerve.getYawAngle() && swerve.getYawAngle() < desiredAngle + 2; 
  }
}