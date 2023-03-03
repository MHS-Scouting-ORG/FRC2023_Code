package frc.robot.commands.PivotCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class TuckedIn extends CommandBase{
    private PivotSubsystem p_subs;
    private int setpoint;

    //PIVOT TUCKED INTO ROBOT PERIMETER 
    public TuckedIn(PivotSubsystem subs){
        p_subs = subs;
        setpoint = -5;
        addRequirements(subs);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("tuckedIn", false);
        if(!p_subs.isTucked()){
        p_subs.newSetpoint(p_subs.getEncoder());
        }
        else{
            p_subs.newSetpoint(setpoint);
           
        }
    }

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("tuckedIn", true);
    }
    
    @Override
    public boolean isFinished(){
        return !p_subs.isTucked() || p_subs.isAtSetPoint();
    }
}