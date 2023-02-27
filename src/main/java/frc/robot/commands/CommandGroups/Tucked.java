// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class Tucked {
  /** Creates a new Tucked. */

public static CommandBase getCommand(PivotSubsystem pivotSub, ElevatorSubsystem elevSub){
  return new ProxyCommand(() -> {
    if (elevSub.getEncoder() < 160) {
      return new TuckedFromBottom(pivotSub, elevSub);
    } else {
      return new TuckedFromTop(pivotSub, elevSub);
    }
  });
}

  // Called when the command is initially scheduled.
}