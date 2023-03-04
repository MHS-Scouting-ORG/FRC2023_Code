package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ElevatorCommands.HighPosition;
import frc.robot.commands.PivotCommands.PivotHighCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class TopNode extends ParallelCommandGroup {

  public TopNode(PivotSubsystem pivotsub, ElevatorSubsystem elevsub) {

    addCommands(
      new HighPosition(elevsub),
      new PivotHighCommand(pivotsub)
    );
  }
}