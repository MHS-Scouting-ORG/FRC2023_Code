package frc.robot.commands.MovementCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightRotatePID extends CommandBase{
    private final SwerveSubsystem swerve;
    private final Timer timer;
    private double desiredAngle; 
    private PIDController turningPID;
    private double currentError;
    private double previousError = 0;

    public LimelightRotatePID(SwerveSubsystem newSwerve){
        swerve = newSwerve;
        timer = new Timer();
        desiredAngle = LimelightHelpers.getTX("limelight"); 
        turningPID = new PIDController(0.001, 0.001,0.0);
        // turningPID.enableContinuousInput(-180, 180); // System is circular;  Goes from -Math.PI to 0 to Math.PI

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

        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        SmartDashboard.putNumber("TX", LimelightHelpers.getTX("limelight"));

        double turningSpeed = turningPID.calculate(0, desiredAngle);

        if (turningSpeed > 0.3){
            turningSpeed = 0.3;
        }
        else if (turningSpeed < -0.3){
            turningSpeed = -0.3;
        }
;
        currentError = desiredAngle;

        if (currentError > 0 && previousError < 0){
            turningPID.reset();
        }
        else if (currentError < 0 && previousError > 0){
            turningPID.reset();
        }

        previousError = currentError;

        SmartDashboard.putNumber("Turning Speed", turningSpeed);
        SmartDashboard.putNumber("Error", currentError);

        //ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed, swerve.getRotation2d());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, turningSpeed);
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
        return false;
    }

}
