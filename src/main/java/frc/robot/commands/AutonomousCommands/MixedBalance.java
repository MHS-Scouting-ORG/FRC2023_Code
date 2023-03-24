package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.Lock;
import frc.robot.commands.MovementCommands.StrafeLeftCommand;
import frc.robot.commands.MovementCommands.StrafeRightCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class MixedBalance extends SequentialCommandGroup {

  public MixedBalance(SwerveSubsystem swerve) {

    addCommands(
      new StrafeRightCommand(swerve, 18 * 4.5, 1.3),
      new Delay(0.15),
      new LevelOut(swerve),
      new Delay(0.15),
      new StrafeLeftCommand(swerve, 35.5, 0.4),
      new Lock(swerve)
    );
  }
}
