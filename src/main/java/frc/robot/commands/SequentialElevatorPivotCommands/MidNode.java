// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SequentialElevatorPivotCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.MidPosition;
import frc.robot.commands.PivotCommands.PivotMiddleCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidNode extends SequentialCommandGroup {
  /** Creates a new StartingConfiguration. */
  public MidNode(PivotSubsystem pivotsub, ElevatorSubsystem elevsub) {
    addCommands(new MidPosition(elevsub), new PivotMiddleCommand(pivotsub));
  }
}