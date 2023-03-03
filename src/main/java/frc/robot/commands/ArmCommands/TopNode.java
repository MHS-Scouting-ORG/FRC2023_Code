package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.HighPosition;
import frc.robot.commands.PivotCommands.PivotHighCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class TopNode extends SequentialCommandGroup {

  // HIGH NODE POSITION
  public TopNode(PivotSubsystem pivotsub, ElevatorSubsystem elevsub) {
    SmartDashboard.putString("Current Command", getName());
    addCommands(new HighPosition(elevsub),(new PivotHighCmd(pivotsub)));
  }
}