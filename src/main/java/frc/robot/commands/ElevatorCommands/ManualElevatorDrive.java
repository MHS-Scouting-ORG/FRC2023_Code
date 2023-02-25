package frc.robot.commands.ElevatorCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
public class ManualElevatorDrive extends CommandBase {
  double speed;  
  ElevatorSubsystem elevatorSubsystem;
  public ManualElevatorDrive(ElevatorSubsystem elevSub,double newSpeed) {
    elevatorSubsystem = elevSub;
    speed = newSpeed;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.disablePid();
  }

  @Override
  public void execute() {
    elevatorSubsystem.setManualSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setManualSpeed(0);
    elevatorSubsystem.enablePid();
    elevatorSubsystem.currentEncValtoSetpoint();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}