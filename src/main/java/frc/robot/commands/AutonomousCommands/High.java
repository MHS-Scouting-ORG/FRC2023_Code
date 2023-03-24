package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.DriveForwardCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class High extends SequentialCommandGroup {

  //SCORE CONE ON HIGH NODE THEN RETURN TO TUCKED POSITION
  public High(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator) {

    addCommands(

      new Claw(claw),

      new Delay(0.15),

      new TuckedFromBottom(pivot, elevator),

      // High goal position (elevator up, pivot out) (parallel cmd)
      new HighAutoPositionParallel(pivot, elevator),

      // Move forward
      new DriveForwardCommand(swerve, 27),

      // Open claw
      new Claw(claw),

      new Delay(0.15),

      new DriveBackwardCommand(swerve, 5),

      new ParallelCommandGroup(
        // Move backward
        new DriveBackwardCommand(swerve, 27), //260
        // Arm in resting position (pivot in, elevator down) 
        new TuckedFromTop(pivot, elevator)
      )
    );
  }
}
