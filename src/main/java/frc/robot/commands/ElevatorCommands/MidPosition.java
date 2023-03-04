package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MidPosition extends CommandBase {
  ElevatorSubsystem elevSub;
  double setPoint;

  // ELEV TO MID POSITION 
  public MidPosition(ElevatorSubsystem elevSubystem) {
    elevSub = elevSubystem;
    setPoint = 100;
    addRequirements(elevSub);
  }

  @Override
  public void initialize() {
    elevSub.init();
  }

  @Override
  public void execute(){
    SmartDashboard.putString("Current Command", getName());

    if(elevSub.topPressed() || elevSub.bottomPressed()){
      elevSub.changeSetpoint(setPoint);
    }

    elevSub.changeSetpoint(setPoint);
    //SmartDashboard.putString("Position:", "Mid");
  }

  @Override
  public void end(boolean interrupted){
    //SmartDashboard.putString("is it working", "yes");

  }

  @Override
  public boolean isFinished() {
    return elevSub.isAtSetpoint(); // stops if the elevator is at the given point
  }
}