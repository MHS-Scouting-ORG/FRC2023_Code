
package frc.robot.commands.MovementCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class StrafeRightCommand extends CommandBase {
  private final SwerveSubsystem swerve; 
  private double desiredEnc; 
  private double multiplier;

  public StrafeRightCommand(SwerveSubsystem newSwerve, double newDesiredEnc, double multiplier) {
    swerve = newSwerve; 
    desiredEnc = newDesiredEnc; 
    this.multiplier = multiplier;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.resetEnc();
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());

    swerve.strafeRight(AutoConsts.DRIVE_TRANSLATION_SPEED*multiplier);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(swerve.getDriveEnc()) > desiredEnc;
  }
}
