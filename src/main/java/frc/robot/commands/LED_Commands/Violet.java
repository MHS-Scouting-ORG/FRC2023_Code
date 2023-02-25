package frc.robot.commands.LED_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class Violet extends CommandBase {
  private Lights lights;

  public Violet(Lights subs) {
    lights = subs;
    addRequirements(subs);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    lights.lavendar();
  }

  @Override
  public void end(boolean interrupted) {
    lights.off();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
