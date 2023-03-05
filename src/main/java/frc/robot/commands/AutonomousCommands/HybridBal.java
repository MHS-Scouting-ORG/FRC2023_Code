package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PitchBalance;
import frc.robot.commands.DriveCommands.Lock;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.FieldRotateRight;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class HybridBal extends SequentialCommandGroup {

  public HybridBal(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator)  {

    //SCORE CONE IN HYBRID GOAL 
    addCommands(

      new Hybrid(swerve, claw, pivot, elevator),

      new DriveBackwardCommand(swerve, 15),

      new FieldRotateRight(swerve, 90),

      // Balance on Charge Station 
      new PitchBalance(swerve), 

      new Lock(swerve)
    );
  }
}
