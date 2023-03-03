package frc.robot.commands.MovementCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateRightCommand extends CommandBase {
  private final SwerveSubsystem swerve; 
  private double desiredAngle; 

  public RotateRightCommand(SwerveSubsystem newSwerve, double newDesiredAngle) {
    swerve = newSwerve;
    desiredAngle = -newDesiredAngle; 

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.resetAutoYaw();
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());
    swerve.rotateRight(AutoConsts.DRIVE_TRANSLATION_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return (swerve.getAutoYaw() >= desiredAngle-2) && (swerve.getAutoYaw() <= desiredAngle+2); 
  }
}
