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

    public FieldRotateLeft(SwerveSubsystem newSwerve, double newDesiredAngle){
        swerve = newSwerve;
        desiredAngle = newDesiredAngle; 
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
        SmartDashboard.putString("CurrentCommand", getName());

        double turningSpeed = -Math.abs(turningPID.calculate(swerve.getYaw(), desiredAngle));

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed, swerve.getRotation2d());

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // Set each module state to wheels
        swerve.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("drive fwd", false); 
        swerve.stopModules();
    }

    @Override
    public boolean isFinished(){
        return turningPID.getPositionError() < 2;
    }

}
