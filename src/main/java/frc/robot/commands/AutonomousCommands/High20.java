package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.DriveForwardCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class High20 extends SequentialCommandGroup {

  //SCORE CONE ON HIGH NODE THEN RETURN TO TUCKED POSITION
  public High20(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator) {

    addCommands(

      //new Launchinator(pivot, claw),

      new TuckedFromBottom(pivot, elevator),

      // High goal position (elevator up, pivot out) (parallel cmd)
      new HighAutoPositionParallel(pivot, elevator),

      // Move forward
      new DriveForwardCommand(swerve, 27),

      // Open claw
      new Claw(claw),

      new Delay(0.5),

      new DriveBackwardCommand(swerve, 5)
    );
  }
}
