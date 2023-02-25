package frc.robot.commands.PivotCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class PivotStartCommand extends CommandBase{ // Pivot PID Command 
    private PivotSubsystem p_subsystem; 
    private Timer timer = new Timer();
    public PivotStartCommand(PivotSubsystem p_subs){ // Pivot PID Constructor 
        p_subsystem = p_subs;
        addRequirements(p_subs);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){ // Executes and runs the Pivot Arm PID
        SmartDashboard.putNumber("Pivot Encoder:", p_subsystem.getEncoder());
        p_subsystem.limitPress();
    }

    @Override
    public void end(boolean interrupted){ // Ends the code when isFinished is true
        SmartDashboard.putString("isFinished?", "yes");
        timer.reset();
        timer.start();
        while(timer.get() < 2){

        }
        p_subsystem.resetEncoder();
    }

    @Override
    public boolean isFinished(){ // Returns true when the code is finished
        return p_subsystem.isTucked();
    }
}