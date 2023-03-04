package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.MovementCommands.DriveForwardCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SideHighPickup extends SequentialCommandGroup {

  public SideHighPickup(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator) {

    // SCORE CONE ON HIGH NODE, PICK UP CONE 
    addCommands(
      new HighMobility(swerve, claw, pivot, elevator), 

      new LowPickUp(pivot, elevator), 

      new DriveForwardCommand(swerve, 20), 

      new Claw(claw), 
      
      new TuckedFromBottom(pivot, elevator)

    );
  }
}
