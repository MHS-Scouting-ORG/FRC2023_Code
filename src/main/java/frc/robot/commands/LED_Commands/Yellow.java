package frc.robot.commands.LED_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class Yellow extends CommandBase {
  private Lights lights;

  public Yellow(Lights subs) {
    lights = subs;
    addRequirements(subs);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    lights.sunflower();
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
