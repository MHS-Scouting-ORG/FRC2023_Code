// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class Test extends CommandBase {
  /** Creates a new Test. */
  ElevatorSubsystem elevSubystem;
  DoubleSupplier speed;
  public Test(ElevatorSubsystem elevSub, DoubleSupplier speed) {
    elevSubystem = elevSub;
    this.speed = speed;
    addRequirements(elevSubystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevSubystem.setManualSpeed(speed.getAsDouble());
    SmartDashboard.getNumber("Manual Speed", speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}