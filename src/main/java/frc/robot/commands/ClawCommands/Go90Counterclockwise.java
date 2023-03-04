package frc.robot.commands.ClawCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawConsts;
import frc.robot.subsystems.ClawSubsystem;

public class Go90Counterclockwise extends CommandBase {

  private ClawSubsystem clawSubsystem;
  private double previousEnc;

  public Go90Counterclockwise(ClawSubsystem claw) {
    clawSubsystem = claw;
    addRequirements(claw);
  }

  @Override
  public void initialize() {
    previousEnc = clawSubsystem.getEncoder();
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());
    clawSubsystem.go90Counterclockwise(previousEnc);
  }

  @Override
  public void end(boolean interrupted) {
    clawSubsystem.stopWrist();
  }

  @Override
  public boolean isFinished() {
    //return Math.abs((previousEnc-ClawConsts.ROTATE_90) - clawSubsystem.getEncoder()) <= 5;
    return ((previousEnc-ClawConsts.ROTATE_90-2) <= clawSubsystem.getEncoder()) && (clawSubsystem.getEncoder() <= (previousEnc-ClawConsts.ROTATE_90+2));
  }
}