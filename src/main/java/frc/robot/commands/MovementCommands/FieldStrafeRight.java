package frc.robot.commands.MovementCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.Constants.SwerveConsts;
import frc.robot.subsystems.SwerveSubsystem;

public class FieldStrafeRight extends CommandBase{
    private final SwerveSubsystem swerve;
    private double desiredEnc; 
    private PIDController turningPID; 

    public FieldStrafeRight(SwerveSubsystem newSwerve, double newDesiredEnc){
        swerve = newSwerve;
        desiredEnc = newDesiredEnc; 
        turningPID = new PIDController(SwerveConsts.KP_TURNING, SwerveConsts.KI_TURNING, SwerveConsts.KD_TURNING);
        turningPID.enableContinuousInput(-Math.PI, Math.PI); // System is circular;  Goes from -Math.PI to 0 to Math.PI

        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        swerve.resetEnc();
    }

    @Override
    public void execute(){
        SmartDashboard.putString("Current Command", getName());

        double turningSpeed = turningPID.calculate(swerve.getYaw(), 0);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, -AutoConsts.DRIVE_TRANSLATION_SPEED, turningSpeed, swerve.getRotation2d());

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
        return Math.abs(swerve.getDriveEnc()) > desiredEnc;
    }

}
