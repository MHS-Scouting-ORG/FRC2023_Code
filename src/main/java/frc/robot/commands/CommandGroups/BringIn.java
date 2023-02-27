// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class BringIn extends CommandBase {
  /** Creates a new BringIn. */
  PivotSubsystem pivotSub;
  ElevatorSubsystem elevSub;
  public BringIn(PivotSubsystem pivotSub, ElevatorSubsystem elevSub) {
    this.pivotSub = pivotSub;
    this.elevSub = elevSub;
    addRequirements(pivotSub, elevSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(elevSub.getEncoder() < 50){
      new TuckedFromBottom(pivotSub, elevSub);
    }
    else{
      new TuckedFromTop(pivotSub, elevSub);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}