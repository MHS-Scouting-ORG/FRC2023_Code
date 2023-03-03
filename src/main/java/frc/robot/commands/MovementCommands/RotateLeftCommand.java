package frc.robot.commands.MovementCommands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateLeftCommand extends CommandBase {
  private final SwerveSubsystem swerve; 
  private double desiredAngle; 

  public RotateLeftCommand(SwerveSubsystem newSwerve, double newDesiredAngle) {
    swerve = newSwerve; 
    desiredAngle = newDesiredAngle; 

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.resetAutoYaw();
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());
    swerve.rotateLeft(AutoConsts.DRIVE_TRANSLATION_SPEED);
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
