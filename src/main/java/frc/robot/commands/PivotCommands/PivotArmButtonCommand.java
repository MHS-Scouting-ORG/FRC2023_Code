package frc.robot.commands.PivotCommands;

import frc.robot.subsystems.PivotSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotArmButtonCommand extends CommandBase{ // Pivot Arm Button Command
    private double speed;
    private PivotSubsystem p_subsystem;
    
    public PivotArmButtonCommand(PivotSubsystem p_subs, double newSpeed){ // Pivot Arm Button Constructor
        p_subsystem = p_subs;
        speed = newSpeed;
        addRequirements(p_subs);
    }

    @Override
    public void initialize(){ // Runs the code when the command is ran 
        p_subsystem.disablePid();
        
   
    }

    @Override
    public void execute(){ // Executes the code and drives the TalonSRX motor to the desired encoder of 3000 when A is pressed
        p_subsystem.setManualSpeed(speed);
       
    }

    @Override
    public void end(boolean interrupted){ // Runs this code when the code is finished 
       p_subsystem.setManualSpeed(0);
       p_subsystem.enablePid();
        p_subsystem.currentEncValtoSetpoint();

    }

    @Override
    public boolean isFinished(){ 
        return false;
    }
}