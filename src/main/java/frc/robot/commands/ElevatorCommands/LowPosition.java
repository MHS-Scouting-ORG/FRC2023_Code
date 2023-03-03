package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class LowPosition extends CommandBase {
  ElevatorSubsystem elevSub;
  double setPoint;

  // ELEV TO LOW POSITION 
  public LowPosition(ElevatorSubsystem elevSubystem) {
    elevSub = elevSubystem;
    setPoint = 6;
    addRequirements(elevSub);
  }

  @Override
  public void initialize() {
    elevSub.init();
  }

  @Override
  public void execute(){
    if(elevSub.topPressed()){
      elevSub.changeSetpoint(setPoint);
    }

    if(elevSub.bottomPressed()){
      elevSub.changeSetpoint(elevSub.getEncoder() + 10);
    }

    elevSub.changeSetpoint(setPoint);
    SmartDashboard.putString("Position:", "low");
  }

  @Override
  public void end(boolean interrupted){

  }

  @Override
  public boolean isFinished() {
    if(elevSub.bottomPressed() || elevSub.isAtSetpoint()){ // stops if the elevator is at the given point
      //SmartDashboard.putBoolean("LowPos fin?", true);
      return true;
    }
     else{ 
      //SmartDashboard.putBoolean("LowPos fin?", false);
      return false;
    }  
  }  
}