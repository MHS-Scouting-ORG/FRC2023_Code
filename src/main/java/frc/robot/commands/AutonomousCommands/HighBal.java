package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.Lock;
import frc.robot.commands.MovementCommands.FieldRotateRight;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class HighBal extends SequentialCommandGroup {

  // SCORE CONE ON HIGH NODE, BALANCE
  public HighBal(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator) {

    addCommands(

      new High(swerve, claw, pivot, elevator),

      new FieldRotateRight(swerve, 88), 

      new MixedBalance(swerve), 

      new Lock(swerve)

    );

  
  }
}
