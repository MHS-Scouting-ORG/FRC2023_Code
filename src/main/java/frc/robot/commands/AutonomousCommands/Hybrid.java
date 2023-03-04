package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.RotateRightCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Hybrid extends SequentialCommandGroup {

  public Hybrid(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator)  {

    //SCORE CONE IN HYBRID GOAL 
    addCommands(

    // start elev in mid position 

      // Arm in hybrid goal position (pivot out, elevator down) 
      new LowPickUp(pivot, elevator),

      // Open claw 
      new Claw(claw),

     // Arm in resting position (pivot in, elevator down) 
      new TuckedFromBottom(pivot, elevator),

      // Move backward
      new DriveBackwardCommand(swerve, 306), 

      // Rotate 90 degrees right so we can strafe right onto Charge Station 
      new RotateRightCommand(swerve, 180)
    );
  }
}
