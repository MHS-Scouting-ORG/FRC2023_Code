package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveForward extends CommandBase{
    private final SwerveSubsystem swerve;
    private double desiredEnc; 

    public DriveForward(SwerveSubsystem newSwerve, double newDesiredEnc){
        swerve = newSwerve;
        desiredEnc = newDesiredEnc; 

        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        swerve.resetEnc();
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("drive fwd", true);
        swerve.driveForward(AutoConsts.DRIVE_TRANSLATION_SPEED);
    }

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("drive fwd", false); 
        swerve.stopModules();
    }

    @Override
    public boolean isFinished(){
        return Math.abs(desiredEnc - swerve.getLeftEncoder()) < 5;
    }

}