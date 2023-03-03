package frc.robot.commands.PivotCommands;

import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotMiddleCmd extends CommandBase{
    private PivotSubsystem p_subsystem; 
    private double setpoint;

    //PIVOT TO MIDDLE SCORING POSITION
    public PivotMiddleCmd(PivotSubsystem p_subs){ // Pivot PID Constructor 
        p_subsystem = p_subs;
        addRequirements(p_subs);
        setpoint = 106;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){ // Executes and runs the Pivot Arm PID
        p_subsystem.newSetpoint(setpoint);
   
    }

    @Override
    public void end(boolean interrupted){ // Ends the code when isFinished is true

    }

    @Override
    public boolean isFinished(){ // Returns true when the code is finished
        return p_subsystem.isAtSetPoint();
    }
}