// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class  ElevatorTest extends CommandBase {
  ElevatorSubsystem elevSub;
  /** Creates a new ElevatorTest. */
  public ElevatorTest(ElevatorSubsystem elevSub) {
    this.elevSub = elevSub;
    addRequirements(elevSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(elevSub.getEncoder() < 50){
      elevSub.changeSetpoint(180);
    }
    else{
      elevSub.changeSetpoint(120);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("Current Command", getName());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevSub.isAtSetpoint();
  }
}