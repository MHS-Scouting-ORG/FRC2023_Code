package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ElevatorCommands.HighPosition;
import frc.robot.commands.PivotCommands.PivotHighCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class HighAutoPositionParallel extends ParallelCommandGroup {

  public HighAutoPositionParallel(PivotSubsystem pivotSub, ElevatorSubsystem elevSub) {

    addCommands(
      new HighPosition(elevSub), 
      new PivotHighCommand(pivotSub)
    );
  }
}