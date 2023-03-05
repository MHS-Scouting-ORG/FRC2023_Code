package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Delay extends CommandBase {
  private final Timer timer; 
  private final double seconds;

  public Delay(Double newSeconds) {
    seconds = newSeconds;
    timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return timer.get() >= seconds;
  }
}
