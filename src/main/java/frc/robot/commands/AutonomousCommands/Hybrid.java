package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Hybrid extends SequentialCommandGroup {

  public Hybrid(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator)  {

    //SCORE CONE IN HYBRID GOAL 
    addCommands(

      new Claw(claw), 

      new Delay(0.5),
      
      new TuckedFromBottom(pivot, elevator),

      // Arm in hybrid goal position (pivot out, elevator down) 
      new LowPickUp(pivot, elevator),

      // Open claw 
      new Claw(claw),

     // Arm in resting position (pivot in, elevator down) 
      new TuckedFromBottom(pivot, elevator)
    );
  }
}
