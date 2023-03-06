package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ZeroPosition extends CommandBase {
  ElevatorSubsystem elevSub;
  double setPoint;

  // ELEV TO RESTING POSITION (OLD STARTING CONFIGURATION)
  public ZeroPosition(ElevatorSubsystem elevSubystem) {
    elevSub = elevSubystem;
    setPoint = 0;
    addRequirements(elevSub);
  }

  @Override
  public void initialize() {
    elevSub.init();
  }

  @Override
  public void execute(){
    SmartDashboard.putString("Current Command", getName());
    elevSub.changeSetpoint(setPoint);
  }

  @Override
  public void end(boolean interrupted){

  }

  
  @Override
  public boolean isFinished() {
    return elevSub.isAtSetpoint(); // stops if the elevator is at the given point
  }
}