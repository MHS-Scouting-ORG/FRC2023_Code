package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveBackward extends CommandBase {
  private final SwerveSubsystem swerve; 
  private double desiredEnc; 

  public DriveBackward(SwerveSubsystem newSwerve, double newDesiredEnc) {
    swerve = newSwerve; 
    desiredEnc = newDesiredEnc; 

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.resetEnc();
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());
    swerve.driveBackward(AutoConsts.DRIVE_TRANSLATION_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return swerve.moveUntil(desiredEnc);
  }
}