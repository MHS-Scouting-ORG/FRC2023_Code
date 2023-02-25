// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SequentialElevatorPivotCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommands.HighPosition;
import frc.robot.commands.PivotCommands.PivotHighCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TopNode extends SequentialCommandGroup {
  /** Creates a new TopNode. */
  public TopNode(PivotSubsystem pivotsub, ElevatorSubsystem elevsub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new HighPosition(elevsub).andThen(new PivotHighCommand(pivotsub)));
  }
}