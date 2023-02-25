package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class LandingGearIn extends CommandBase {
  private SwerveSubsystem swerve;

  public LandingGearIn(SwerveSubsystem subs) {
    swerve = subs;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.wheelsIn();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
