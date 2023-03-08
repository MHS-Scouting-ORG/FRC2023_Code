package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class TankEndgame extends CommandBase {
  private SwerveSubsystem swerve;
  private DoubleSupplier left, right;

  public TankEndgame(SwerveSubsystem s, DoubleSupplier l, DoubleSupplier r) {
    swerve = s;
    left = l;
    right = r;
    addRequirements(s);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.wheelsOut();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());
    swerve.setEndgame(left.getAsDouble(), right.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
