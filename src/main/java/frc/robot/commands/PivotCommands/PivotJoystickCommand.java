// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.PivotCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class PivotJoystickCommand extends CommandBase {
  /** Creates a new Test. */
  PivotSubsystem pivotarmsubsystem;
  DoubleSupplier speed;
  public PivotJoystickCommand(PivotSubsystem pivotSub, DoubleSupplier speed) {
    pivotarmsubsystem = pivotSub;
    this.speed = speed;
    addRequirements(pivotarmsubsystem);
  }

  @Override
  public void initialize() {
    pivotarmsubsystem.disablePid();
  }

  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());
    pivotarmsubsystem.setManualSpeed(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    pivotarmsubsystem.enablePid();
    pivotarmsubsystem.currentEncValtoSetpoint();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}