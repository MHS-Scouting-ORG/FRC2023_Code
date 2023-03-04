package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PitchBalance;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.RotateLeftCommand;
import frc.robot.commands.MovementCommands.RotateRightCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class HybridBal extends SequentialCommandGroup {

  public HybridBal(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator)  {
    // Timer time = new Timer(); 
    // time.restart();
    // time.start();

    //SCORE CONE IN HYBRID GOAL 
    addCommands(

      new Hybrid(swerve, claw, pivot, elevator),

      new DriveBackwardCommand(swerve, 20)

      //new RotateRightCommand(swerve, 90),

      // Balance on Charge Station 
      //new PitchBalance(swerve/* , time.get() */)
    );
  }
}
