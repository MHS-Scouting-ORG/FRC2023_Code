package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.HighAutoPositionParallel;
import frc.robot.commands.ArmCommands.TuckedFromBottom;
import frc.robot.commands.ArmCommands.TuckedFromTop;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.DriveForwardCommand;
import frc.robot.commands.MovementCommands.RotateRightCommand;
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
      new DriveForwardCommand(swerve, 27),

      // Open claw
      new Claw(claw),

      new ParallelCommandGroup(
      // Move backward
      new DriveBackwardCommand(swerve, 100),

      // Arm in resting position (pivot in, elevator down) 
      new TuckedFromTop(pivot, elevator)
      ), 

      new RotateRightCommand(swerve, 180)


    );
  }
}
