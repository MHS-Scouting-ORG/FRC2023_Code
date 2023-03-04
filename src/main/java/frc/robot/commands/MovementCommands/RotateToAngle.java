package frc.robot.commands.MovementCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.Constants.SwerveConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateToAngle extends CommandBase {
  private SwerveSubsystem swerve;
  private double desiredAngle;
  private double current;
  public RotateToAngle(SwerveSubsystem s, double angle) {
    swerve = s;
    desiredAngle = angle;
    current = swerve.getYawAngle();
    addRequirements(s);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(swerve.getYawAngle() < 180){
      swerve.rotateRight(AutoConsts.DRIVE_ROTATION_SPEED);
    } else{
      swerve.rotateLeft(AutoConsts.DRIVE_ROTATION_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(swerve.getYawAngle() - current) >= desiredAngle;  //Math.abs(swerve.getYawAngle() - desiredAngle) < 5;
  }
}
