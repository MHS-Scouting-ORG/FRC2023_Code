package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PivotCommands.*;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.Autonomous.*;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class High extends SequentialCommandGroup {

  //SCORE CONE ON HIGH NODE 
  public High(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator) {

    addCommands(

      new TuckedFromBottom(pivot, elevator),

      // High goal position (elevator up, pivot out) (parallel cmd)
      new HighAutoPositionParallel(pivot, elevator),

      // Move forward
      new DriveForward(swerve, 27),

      // Open claw
      new Claw(claw),

      new ParallelCommandGroup(
      // Move backward
      new DriveBackward(swerve, 100),

      // Arm in resting position (pivot in, elevator down) 
      new TuckedFromTop(pivot, elevator)
      )

      //new RotateRight(swerve, 180)


    );
  }
}