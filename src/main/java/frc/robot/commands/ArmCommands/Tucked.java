package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class Tucked extends CommandBase {
  PivotSubsystem pivotSub;
  ElevatorSubsystem elevSub;

  // TUCKED/RESTING POSITION
  // use Tucked.getCommand()
  public Tucked(PivotSubsystem pivotSub, ElevatorSubsystem elevSub){
    this.pivotSub = pivotSub;
    this.elevSub = elevSub;
    addRequirements(pivotSub, elevSub);

  }
  public static CommandBase getCommand(PivotSubsystem pivotSub, ElevatorSubsystem elevSub) {

    return new ProxyCommand(() -> {
      if (elevSub.getEncoder() < 160) {
        return new TuckedFromBottom(pivotSub, elevSub);
      } else {
        return new TuckedFromTop(pivotSub, elevSub);
      }
    });
  }

  public void initialize(){
    
  }

 
}