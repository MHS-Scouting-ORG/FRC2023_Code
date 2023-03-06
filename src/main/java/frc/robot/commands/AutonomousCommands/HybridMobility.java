package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.CommandGroups.LowPickUp;
import frc.robot.commands.CommandGroups.TuckedFromBottom;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.DriveForwardCommand;
import frc.robot.commands.MovementCommands.RotateLeftCommand;
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

      new RotateLeftCommand(swerve, 163),

      new LowPickUp(pivot, elevator),

      new DriveForwardCommand(swerve, 18),

      new Claw(claw),

      new Delay(0.5),

      new TuckedFromBottom(pivot, elevator)
    );
  }
}
