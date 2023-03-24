package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class PitchBalance extends CommandBase {
  private final SwerveSubsystem swerve;
  private boolean readyToEnd;
  //private double time;

  // MOVE UNTIL 13.5 IS READ A SECOND TIME (WHEN IT'S GOING DOWN FROM 15 DEGREES)
  public PitchBalance(SwerveSubsystem newSwerve/* , double newTime */) {
    swerve = newSwerve;
    readyToEnd = false;
    //time = newTime; 

    addRequirements(swerve);
  }

  public void display() {
    SmartDashboard.putString("Current Command", getName());
    SmartDashboard.putBoolean("[A] R2E", readyToEnd);
  }

  @Override
  public void initialize() {
    // Reset boolean to false
    readyToEnd = false;
  }

  @Override
  public void execute() {
    display();

    // Strafes right onto charge station
    swerve.strafeRight(AutoConsts.DRIVE_TRANSLATION_SPEED * 0.8);
    
    // Checks if pitch is greater than -13.5
    if (swerve.getPitch() < -15){
      readyToEnd = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Current Command", "Lock");
    // Stop then lock drive
    swerve.lock();
    //swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    // Checks a second time if pitch is greater than -13.5, ends if true
    //return (readyToEnd && swerve.getPitch() > -13 || Timer.getMatchTime() < 1)/*  || time > 14 */;
    return (readyToEnd && swerve.getPitch() > -11); //|| Timer.getMatchTime()<1.5; //|| (-5<swerve.getPitch() && swerve.getPitch()<5 && Timer.getMatchTime() < 1);
  }
}
