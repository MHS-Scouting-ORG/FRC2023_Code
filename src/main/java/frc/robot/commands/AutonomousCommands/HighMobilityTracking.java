// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CommandGroups.TuckedFromBottom;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.FieldRotateLeft;
import frc.robot.commands.MovementCommands.LimelightRotate;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class HighMobilityTracking extends SequentialCommandGroup {
  // SCORE ON HIGH, MOBILITY
  public HighMobilityTracking(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot,
      ElevatorSubsystem elevator) {

    addCommands(

        new High20(swerve, claw, pivot, elevator),

        new ParallelCommandGroup(
            new DriveBackwardCommand(swerve, 170),
            new TuckedFromBottom(pivot, elevator)),

        new FieldRotateLeft(swerve, 160),

        new Delay(0.2),

        new LimelightRotate(swerve)

        // new ParallelCommandGroup(
        //     new DriveForwardCommand(swerve, 230),
        //     new LowPickUp(pivot, elevator)),

        // new Claw(claw),

        // new Delay(0.5),

        // new TuckedFromBottom(pivot, elevator)
        );
  }
}
