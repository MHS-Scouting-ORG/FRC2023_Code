package frc.robot.commands.ClawCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ManualClaw extends CommandBase{
    private ClawSubsystem claw;
    private DoubleSupplier supplier;

    public ManualClaw(ClawSubsystem c, DoubleSupplier d){
        claw = c;
        supplier = d;
        addRequirements(c);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        SmartDashboard.putString("Current Command", getName());
        claw.manualRotate(supplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
        claw.stopWrist();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
