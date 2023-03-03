package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class HighPosition extends CommandBase {
  ElevatorSubsystem elevSub;
  double setPoint;

  // ELEV TO HIGH POSITION 
  public HighPosition(ElevatorSubsystem elevSubystem) {
    elevSub = elevSubystem;
    setPoint = 180;
    addRequirements(elevSub);
  }

  @Override
  public void initialize() {
    elevSub.init();
  }

  @Override
  public void execute(){
    if(elevSub.topPressed()){
      elevSub.changeSetpoint(elevSub.getEncoder() - 10);
    }

    elevSub.changeSetpoint(setPoint);
    SmartDashboard.putString("Position:", "High");
  }

  @Override
  public void end(boolean interrupted){
  }

  
  @Override
  public boolean isFinished() {
    if(elevSub.topPressed() || elevSub.isAtSetpoint()){
     return true;
    } // stops if the elevator is at the given point
    else{
      return false;
    }
  }
}