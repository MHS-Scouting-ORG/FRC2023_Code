package frc.robot.commands.MovementCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class FieldRotateRight extends CommandBase{
    private final SwerveSubsystem swerve;
    private final Timer timer;
    private double desiredAngle; 
    private PIDController turningPID; 

    public FieldRotateRight(SwerveSubsystem newSwerve, double newDesiredAngle){
        swerve = newSwerve;
        timer = new Timer();
        desiredAngle = newDesiredAngle; 
        turningPID = new PIDController(SwerveConsts.KP_TURNING, 0.0,0.0);
        turningPID.enableContinuousInput(-Math.PI, Math.PI); // System is circular;  Goes from -Math.PI to 0 to Math.PI

        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        SmartDashboard.putString("CurrentCommand", getName());
        SmartDashboard.putBoolean("rotating", true);
        SmartDashboard.putNumber("Rotation Timer", timer.get());

        double turningSpeed = turningPID.calculate(swerve.getYaw(), desiredAngle);

        if (turningSpeed > 0.5){
            turningSpeed = 0.5;
        }
        else if (turningSpeed < -0.5){
            turningSpeed = -0.5;
        }

        SmartDashboard.putNumber("Turning Speed", turningSpeed);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed, swerve.getRotation2d());

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // Set each module state to wheels
        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("rotating", false); 
        swerve.stopModules();
        timer.stop();
    }

    @Override
    public boolean isFinished(){
        return Math.abs(desiredAngle - swerve.getYaw()) < 2 && timer.get() > 2;
    }

}
