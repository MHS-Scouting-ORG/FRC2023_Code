package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class IncrementBalanceCommand extends CommandBase {
  private final SwerveSubsystem swerve;
  private final Timer timer, autoTimer; 
  private int counter;

  public IncrementBalanceCommand(SwerveSubsystem newSwerve) {
    swerve = newSwerve;
    timer = new Timer();
    autoTimer = new Timer(); 

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    autoTimer.start();
    swerve.resetEnc();
    counter = 0;
  }

  @Override
  public void execute() {

    switch(counter) {
      // drive a little til robot pitch(navx roll) is 12 degrees 
      case 0: 
      if (swerve.getRoll() >= AutoConsts.initialPitch) {
        swerve.stopModules();
        swerve.resetEnc();
        counter++; 
      } else {
        swerve.driveForward(AutoConsts.driveTranslationSlowSpeed);
      }
      break; 

      /* 
      // drive 3 enc counts (about 2 inches???)
      case 1: 
      if (swerve.getEnc() > AutoConsts.incrementalEncValue) {
        swerve.stopModules();
        counter++; 
      } else {
        swerve.driveForward(AutoConsts.driveTranslationSlowSpeed);
      }
      break; 

      // wait 2 seconds, if balanced, stop
      case 2: 
      timer.start();
      if (timer.get() > 2) {
        //check roll 
        if (swerve.getRoll() < 6) {
          swerve.stopModules();
          counter++; 
        } else {
          timer.stop();
          timer.reset();
          counter = 1; 
        }

      }
      break; */

      default:
    }

/* 
    switch (counter) {
      case 0: //drive until -12 deg (initial startup)
        if (swerve.getRoll() <= AutoConsts.initialPitch) {
          swerve.stopModules();
          swerve.resetEnc();
          counter++; 
        } else {
          swerve.driveBackward();
        }
      break; 

      case 1: // drive a little
      swerve.resetEnc();
        if (swerve.getEnc() < AutoConsts.incrementalEncValue) {
          swerve.stopModules();
          counter++;
        } else {
          swerve.driveBackward();
        }

      break;
      
      case 2: // wait and check if roll goes below 10??? deg and go back to case 1 if not 
      timer.start();
      if (timer.get() > 2) { //wait 2 sec 

        //if level, stop 
        if (swerve.getRoll() > AutoConsts.balanceThreshhold) {
          counter++;
        } else {
          counter = 1;
        }

        timer.stop();
        timer.reset();

      }

      break;
      
      
    }

    /* * * Smart Dashboard * * */
    SmartDashboard.putString("Current Command", getName());
    SmartDashboard.putNumber("Counter", counter);
    SmartDashboard.putNumber("Auto Timer", autoTimer.get());
    SmartDashboard.putNumber("Timer", timer.get());
    SmartDashboard.putNumber("drive enc", swerve.getDriveEnc()); 
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Current Command", "LOCKED!!!");
    swerve.stopModules();
    swerve.lock();
  }

  @Override
  public boolean isFinished() {
    //FIXME change pls; actually ends at 3 
    //start off slower; align wheels at init
    return counter == 3 || autoTimer.get() >= 14.5;
  }
}