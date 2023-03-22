package frc.robot.commands.MovementCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignCommand extends CommandBase {

  private SwerveSubsystem swerve;

  public AlignCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    LimelightHelpers.align();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return -0.75 < LimelightHelpers.getTX("limelight") && LimelightHelpers.getTX("limelight") < 0.75;
  }
}
