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
import frc.robot.commands.MovementCommands.FieldRotateRight;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class HighMobility extends SequentialCommandGroup {
  //SCORE ON HIGH, MOBILITY
  public HighMobility(SwerveSubsystem swerve, ClawSubsystem claw, PivotSubsystem pivot, ElevatorSubsystem elevator) {

    addCommands(

      new High(swerve, claw, pivot, elevator),

      new DriveBackwardCommand(swerve, 230), 

      // new RotateLeftCommand(swerve, 175),
      // new FieldRotateRight(swerve, 178),

      // new ParallelCommandGroup(
      //   new LowPickUp(pivot, elevator),
      //   new DriveForwardCommand(swerve, 18)
      // ),

      // new Claw(claw),

      new Delay(0.5),

      new TuckedFromBottom(pivot, elevator)
    );
  }
}
