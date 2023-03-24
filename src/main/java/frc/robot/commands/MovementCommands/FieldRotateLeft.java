package frc.robot.commands.MovementCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class FieldRotateLeft extends CommandBase{
    private final SwerveSubsystem swerve;
    private double desiredAngle; 
    private PIDController turningPID;
    private double currentError;
    private double previousError = 0;

    public FieldRotateLeft(SwerveSubsystem newSwerve, double newDesiredAngle){
        swerve = newSwerve;
        desiredAngle = -newDesiredAngle; 
        turningPID = new PIDController(0.01, 0.01,0.0);
        // turningPID.enableContinuousInput(-180, 180); // System is circular;  Goes from -Math.PI to 0 to Math.PI

        addRequirements(swerve);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        SmartDashboard.putString("Current Command", getName());

        double currentYaw = swerve.getYaw();

        double turningSpeed = turningPID.calculate(currentYaw, desiredAngle);

        if (turningSpeed > 0.5){
            turningSpeed = 0.5;
        }
        else if (turningSpeed < -0.5){
            turningSpeed = -0.5;
        }

        currentError = desiredAngle - swerve.getYaw();

        if (currentError > 0 && previousError < 0){
            turningPID.reset();
        }
        else if (currentError < 0 && previousError > 0){
            turningPID.reset();
        }

        previousError = currentError;

        //ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed, swerve.getRotation2d());

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, turningSpeed);
        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // Set each module state to wheels
        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        swerve.stopModules();
    }

    @Override
    public boolean isFinished(){
        return Math.abs(desiredAngle - swerve.getYaw()) < 5;
    }

}
