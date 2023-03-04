package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.LowPosition;
import frc.robot.commands.ElevatorCommands.MidPosition;
import frc.robot.commands.PivotCommands.PivotLowCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class LowPickUp extends SequentialCommandGroup {

  public LowPickUp(PivotSubsystem pivotSub, ElevatorSubsystem elevSub) {

    addCommands(
      new MidPosition(elevSub), 
      new PivotLowCommand(pivotSub), 
      new LowPosition(elevSub)
    );
  }
}