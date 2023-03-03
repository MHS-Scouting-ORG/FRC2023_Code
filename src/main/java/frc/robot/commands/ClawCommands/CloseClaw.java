package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

//TOGGLE CLAW (CLOSE AND OPEN)
public class CloseClaw extends CommandBase {
  private ClawSubsystem clawSubsystem;

  public CloseClaw(ClawSubsystem claw) {
    clawSubsystem = claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    clawSubsystem.closeClaw();
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