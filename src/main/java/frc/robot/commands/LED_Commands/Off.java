package frc.robot.commands.LED_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class Off extends CommandBase {
  private Lights lights;

  public Off(Lights subs) {
    lights = subs;
    addRequirements(subs);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    lights.off();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
