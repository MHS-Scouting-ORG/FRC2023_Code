package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PivotCommands.PivotLowCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class Launchinator extends SequentialCommandGroup {

  public Launchinator(PivotSubsystem pivot, ClawSubsystem claw) {

    // LAUNCH-INATE (RELEASE) RUBBER BAND 
    addCommands(
      new PivotLowCommand(pivot) 

      //new ToStartingPosition(claw)
    );
  }
}
