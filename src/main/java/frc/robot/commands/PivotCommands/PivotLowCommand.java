package frc.robot.commands.PivotCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class PivotLowCommand extends CommandBase{
    private PivotSubsystem p_subs;

    public PivotLowCommand(PivotSubsystem subs){
        p_subs = subs;
        addRequirements(subs);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Pivot Encoder: ", p_subs.getEncoder());
        p_subs.newSetpoint(63);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}