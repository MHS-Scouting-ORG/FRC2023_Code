package frc.robot.commands.MovementCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightRotate extends CommandBase{
    private final SwerveSubsystem swerve;
    private final Timer timer;

    public LimelightRotate(SwerveSubsystem newSwerve){
        swerve = newSwerve;
        timer = new Timer();

        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("rotating", true);
        
        if (LimelightHelpers.getTX("limelight") < -1.5){
            swerve.rotateLeft(0.1);
        }
        else if(LimelightHelpers.getTX("limelight") > 1.5){
            swerve.rotateRight(0.1);
        }

        SmartDashboard.putNumber("x-offset", LimelightHelpers.getTX("limelight"));
    }

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("rotating", false); 
        swerve.stopModules();
        timer.stop();
    }

    @Override
    public boolean isFinished(){
        return (LimelightHelpers.getTX("limelight") > -1.5 && LimelightHelpers.getTX("limelight") < 1.5) || LimelightHelpers.getTX("limelight") == 0;
    }

}
