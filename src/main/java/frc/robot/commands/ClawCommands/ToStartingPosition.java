package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ToStartingPosition extends CommandBase {

  private ClawSubsystem clawSubsystem;

  public ToStartingPosition(ClawSubsystem claw) {
    clawSubsystem = claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());
    clawSubsystem.startingPosition();
  }

  @Override
  public void end(boolean interrupted) {
    clawSubsystem.stopWrist();
  }

  @Override
  public boolean isFinished() {
    return -2 <= clawSubsystem.getEncoder() && clawSubsystem.getEncoder() <= 2;
  }
}
