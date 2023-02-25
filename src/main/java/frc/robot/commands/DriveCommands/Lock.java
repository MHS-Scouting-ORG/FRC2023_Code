package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Lock extends CommandBase{
    
    private SwerveSubsystem swerve;

    public Lock(SwerveSubsystem subs){
        swerve = subs;

        addRequirements(subs);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        swerve.lock();
        
    }

    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished(){
         return false;
    }
}
