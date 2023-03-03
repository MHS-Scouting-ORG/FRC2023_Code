package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ElevatorCommands.MidPosition;
import frc.robot.commands.PivotCommands.TuckedIn;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class TuckedFromTop extends ParallelCommandGroup {

  // TUCK ARM IN WHEN ELEV CLEARS BUMPER 
  public TuckedFromTop(PivotSubsystem pivotSub, ElevatorSubsystem elevSub) {

    SmartDashboard.putString("Current Command", getName());
    addCommands(new TuckedIn(pivotSub), new MidPosition(elevSub));
  }
}