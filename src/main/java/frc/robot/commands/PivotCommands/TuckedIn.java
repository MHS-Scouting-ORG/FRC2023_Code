package frc.robot.commands.PivotCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class TuckedIn extends CommandBase{
    private PivotSubsystem p_subs;
    private int setpoint;

    public TuckedIn(PivotSubsystem subs){
        p_subs = subs;
        setpoint = 0;
        addRequirements(subs);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if(!p_subs.isTucked()){
        p_subs.newSetpoint(p_subs.getEncoder());
        }
        else{
            p_subs.newSetpoint(setpoint);
           
        }
    }

    @Override
    public void end(boolean interrupted){
    }
    
    @Override
    public boolean isFinished(){
        return !p_subs.isTucked();
    }
}