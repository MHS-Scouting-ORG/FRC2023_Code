package frc.robot.commands; 

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDBalanceCommand extends CommandBase {
  private final SwerveSubsystem swerve; 
  //redo values on swerve. These are for Sparky
  private final PIDController pidController = new PIDController(0.3, 00017, 0.028); 

  public PIDBalanceCommand(SwerveSubsystem newSwerve) {
    swerve = newSwerve; 

    addRequirements(swerve);
  }

  //calculates speed to apply to chassis 
  //mac speed 0.6
  public double outputSpeed() {
    double speed; 
    if (swerve.getRoll() < 2.5 && swerve.getRoll() > -3.5) {
      speed = 0; 
    } else {
      speed = pidController.calculate(swerve.getRoll(), 0 ); 
    }

    if (speed > 0.6) {
      return 0.6; 
    } else if (speed < -0.6) {
      return -0.6; 
    } else {
      return speed; 
    }
  }

  //resets PID every time robot moves past setpoint 
  public void resetI() {
    double prevPosition = 0; 
    double currentPosition = pidController.getPositionError(); 

    if (currentPosition > 0 && prevPosition < 0) {
      pidController.reset();
    } else if (currentPosition <0 && prevPosition > 0) {
      pidController.reset();
    } 

    prevPosition = pidController.getPositionError(); 
  }

  @Override
  public void initialize() {
    swerve.stopModules();
  } 

  @Override
  public void execute() {
    swerve.pidDrive(outputSpeed(), 0, 0);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
