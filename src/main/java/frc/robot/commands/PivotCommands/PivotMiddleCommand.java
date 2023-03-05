package frc.robot.commands.PivotCommands;

import frc.robot.subsystems.PivotSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotMiddleCommand extends CommandBase{
    private PivotSubsystem p_subsystem; 
    private double setpoint;

    public PivotMiddleCommand(PivotSubsystem p_subs){ // Pivot PID Constructor 
        p_subsystem = p_subs;
        addRequirements(p_subs);
        setpoint = 102;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){ // Executes and runs the Pivot Arm PID
        SmartDashboard.putString("Current Command", getName());
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