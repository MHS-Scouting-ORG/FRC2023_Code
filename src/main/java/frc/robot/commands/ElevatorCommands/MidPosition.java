package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MidPosition extends CommandBase {
  ElevatorSubsystem elevSub;
  double setPoint;
  public MidPosition(ElevatorSubsystem elevSubystem) {
    elevSub = elevSubystem;
    setPoint = 120;
    addRequirements(elevSub);
  }

  @Override
  public void initialize() {
    elevSub.init();
  }

  @Override
  public void execute(){
    if(elevSub.topPressed() || elevSub.bottomPressed()){
      elevSub.changeSetpoint(setPoint);
    }

    elevSub.changeSetpoint(setPoint);
    SmartDashboard.putString("Position:", "Mid");
  }

  @Override
  public void end(boolean interrupted){

  }

  @Override
  public boolean isFinished() {
    return elevSub.isAtSetpoint(); // stops if the elevator is at the given point
  }
}