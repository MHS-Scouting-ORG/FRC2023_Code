package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.RotateRightCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class HybridMobility extends SequentialCommandGroup {

  // SCORES HYBRID, MOBILITY 
  public HybridMobility(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator) {

    addCommands(
      new Hybrid(swerve, claw, pivot, elevator), 
      
      new DriveBackwardCommand(swerve, 230), 
      
      new RotateRightCommand(swerve, 180)
    );
  }
}
