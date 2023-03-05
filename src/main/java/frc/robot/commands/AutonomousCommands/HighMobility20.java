// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.CommandGroups.LowPickUp;
import frc.robot.commands.CommandGroups.TuckedFromBottom;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.DriveForwardCommand;
import frc.robot.commands.MovementCommands.RotateLeftCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class HighMobility20 extends SequentialCommandGroup {
  //SCORE ON HIGH, MOBILITY
  public HighMobility20(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator) {

    addCommands(

      new High20(swerve, claw, pivot, elevator),

      new ParallelCommandGroup(
        new DriveBackwardCommand(swerve, 257),
        new TuckedFromBottom(pivot, elevator)
      ),

      new ParallelCommandGroup(
        new RotateLeftCommand(swerve, 163),
        //new FieldRotateLeft(swerve, 180),
        new LowPickUp(pivot, elevator)
      ),

      new DriveForwardCommand(swerve, 18),

      new Claw(claw),

      new Delay(0.5),

      new TuckedFromBottom(pivot, elevator)
    );
  }
}
