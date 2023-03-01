package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateLeft extends CommandBase {
  private final SwerveSubsystem swerve; 
  private double desiredAngle; 

  public RotateLeft(SwerveSubsystem newSwerve, double newDesiredAngle) {
    swerve = newSwerve; 
    desiredAngle = swerve.getAngle() - newDesiredAngle; 

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
    return (swerve.getAngle() >= desiredAngle-2) && (swerve.getAngle() <= desiredAngle+2); 
  }
}