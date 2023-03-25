// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.Lock;
import frc.robot.commands.MovementCommands.DriveForwardCommand;
import frc.robot.commands.MovementCommands.FieldRotateRight;
import frc.robot.commands.MovementCommands.StrafeRightCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedHighBalEnc extends SequentialCommandGroup {
  /** Creates a new HighBalEnc. */
  public RedHighBalEnc(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new High(swerve, claw, pivot, elevator),
      new DriveForwardCommand(swerve, 15),
      new Delay(0.5),
      //new DriveBackwardCommand(swerve, 145),

      new FieldRotateRight(swerve, 90), 

      new StrafeRightCommand(swerve, 162, 0.8),

      new Lock(swerve)
    );
  }
}
