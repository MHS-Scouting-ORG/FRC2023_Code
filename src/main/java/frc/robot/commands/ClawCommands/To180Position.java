package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConsts;
import frc.robot.subsystems.ClawSubsystem;

public class To180Position extends CommandBase {

  private ClawSubsystem clawSubsystem;

  public To180Position(ClawSubsystem claw) {
    clawSubsystem = claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    clawSubsystem.rotateTo180();
  }

  @Override
  public void end(boolean interrupted) {
    clawSubsystem.stopWrist();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(ClawConsts.ROTATION_TO_180_ENC - clawSubsystem.getEncoder()) <= 10;
    //return 190 <= clawSubsystem.getEncoder() && clawSubsystem.getEncoder() <= 210;
  }
}
