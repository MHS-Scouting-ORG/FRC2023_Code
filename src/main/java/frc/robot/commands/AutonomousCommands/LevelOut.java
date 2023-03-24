package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class LevelOut extends CommandBase {
  private SwerveSubsystem swerve;

  public LevelOut(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(swerve.getPitch() < 1.8){
      swerve.strafeRight(0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //swerve.stopModules();
    swerve.lock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.getPitch() > 2.3; // 2.2
  }
}
