package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.MidPosition;
import frc.robot.commands.PivotCommands.TuckedIn;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class TuckedFromBottom extends SequentialCommandGroup {

  // TUCK ARM IN WHEN ELEV IS AT BOTTOM 
  public TuckedFromBottom(PivotSubsystem pivotSub, ElevatorSubsystem elevSub) {

    addCommands(new MidPosition(elevSub), new TuckedIn(pivotSub));
  }
}