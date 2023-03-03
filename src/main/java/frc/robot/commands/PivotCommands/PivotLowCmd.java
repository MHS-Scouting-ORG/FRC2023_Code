package frc.robot.commands.PivotCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class PivotLowCmd extends CommandBase{
    private PivotSubsystem p_subs;

    //PIVOT RESTING ON BUMPER 
    public PivotLowCmd(PivotSubsystem subs){
        p_subs = subs;
        addRequirements(subs);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Pivot Encoder: ", p_subs.getEncoder());
        p_subs.newSetpoint(45);
    }

    @Override
    public boolean isFinished(){
        return p_subs.isAtSetPoint();
    }

}