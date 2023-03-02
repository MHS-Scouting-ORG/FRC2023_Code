package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Rotate180 extends CommandBase {
  private SwerveSubsystem swerve;
  
  public Rotate180(SwerveSubsystem s) {
    swerve = s;
    addRequirements(s);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
