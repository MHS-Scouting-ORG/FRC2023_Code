package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

//TOGGLE CLAW (CLOSE AND OPEN)
public class Claw extends CommandBase {
  private ClawSubsystem clawSubsystem;

  public Claw(ClawSubsystem claw) {
    clawSubsystem = claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    clawSubsystem.toggle();
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