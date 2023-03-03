package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveModule extends SubsystemBase{
    private CANSparkMax turningMotor;
    private CANSparkMax drivingMotor;

    private CANCoder absoluteEncoder;

    private RelativeEncoder turningEnc;
    private RelativeEncoder drivingEnc;

    private PIDController turningPID;
    // private PIDController drivingPID;

    private double encoderOffset;
    private boolean encoderReversed;

     /////////////////////
     //   CONSTRUCTOR   //
     /////////////////////

    public SwerveModule(int turnPort, int drivePort, int cancoderPort, double encoderOffset, boolean encoderReversed, boolean driveReversed, boolean turnReversed){
        turningMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
        drivingMotor = new CANSparkMax(drivePort, MotorType.kBrushless);

        absoluteEncoder = new CANCoder(cancoderPort);
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        turningEnc = turningMotor.getEncoder();
        drivingEnc = drivingMotor.getEncoder();

        turningPID = new PIDController(SwerveConsts.kp_turning, SwerveConsts.ki_turning, SwerveConsts.kd_turning);
        turningPID.enableContinuousInput(-Math.PI, Math.PI); // System is circular;  Goes from -Math.PI to 0 to Math.PI

        // drivingPID = new PIDController(SwerveConsts.kp_driving, SwerveConsts.ki_driving, SwerveConsts.kd_driving);

        this.encoderOffset = encoderOffset;
        this.encoderReversed = encoderReversed;
        
        turningEnc.setPositionConversionFactor(SwerveConsts.turningEncoderRotationConversion);
        turningEnc.setVelocityConversionFactor(SwerveConsts.turningEncoderSpeedConversion);
        drivingEnc.setPositionConversionFactor(SwerveConsts.driveEncoderRotationConversion);
        drivingEnc.setVelocityConversionFactor(SwerveConsts.driveEncoderSpeedConversion);

        drivingMotor.setInverted(driveReversed);
        turningMotor.setInverted(turnReversed);

        drivingMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        resetEncoders();

    }

     /////////////////////
     //   GET METHODS   //
     /////////////////////

    /* * * ENCODER VALUES * * */

    public double getDrivePosition(){
        //return (drivingEnc.getPosition() % 900) * 360;
        return drivingEnc.getPosition();
    }

    public double getTurningPosition(){
        //return (turningEnc.getPosition() % 900) / 900 * 360;
        return turningEnc.getPosition() + encoderOffset;
    }

    //absolute encoder in radians 
    public double getAbsoluteEncoder(){
        double angle = Math.toRadians(absoluteEncoder.getAbsolutePosition());
        //double angle = (absoluteEncoder.getAbsolutePosition() * 2.0 * Math.PI) - encoderOffset; // in radians
        if(encoderReversed){
            return (angle * -1);
        } else {
            return angle;
        }
    }

    /* * * SPEED VALUES * * */

    public double getDriveSpeed(){
        return drivingEnc.getVelocity();
    }

    public double getTurningSpeed(){
        return turningEnc.getVelocity();
    }

    //get the state of the module
    public SwerveModuleState getState(){
        // Rotation2d = rotation represented by a point on the unit circle
        // Rotation2d(double) => constructs a Rotation2d given the angle in radians
        
        // SwerveModuleState = state of a swerve module
        // SwerveModuleState(speed (in meters per second), angle of module (Using Rotation2d))
        return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getAbsoluteEncoder()));
    }

    /////////////////////
    //   SET METHODS   //
    /////////////////////

    // set turning enc to value of absolute encoder
    public void resetEncoders(){
        drivingEnc.setPosition(0);
        turningEnc.setPosition(getAbsoluteEncoder());
    }

    //sets the desired state of the module (for left joystick)
    public void setDesiredState(SwerveModuleState state){
        // To make keep robot from going back to 0 position
        if(Math.abs(state.speedMetersPerSecond) < 0.1){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);

        // set speed
        drivingMotor.set(state.speedMetersPerSecond / SwerveConsts.maxSpeed_mps * SwerveConsts.voltage); 
        turningMotor.set(turningPID.calculate(getAbsoluteEncoder(), state.angle.getRadians()));

        // Print to SmartDashboard
        //SmartDashboard.putString("Swerve["+absoluteEncoder.getDeviceID()+"] state", state.toString());
        SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] drive speed", getDriveSpeed());
        //SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] angle", getAbsoluteEncoder());
    }

    //sets the desired angle of the module (for right joystick)
    public void setAngle(SwerveModuleState state){

        state = SwerveModuleState.optimize(state, getState().angle);

        // set speed
        drivingMotor.set(0); 
        turningMotor.set(turningPID.calculate(getAbsoluteEncoder(), state.angle.getRadians()));

        // Print to SmartDashboard
        //SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] desired enc", state.angle.getRadians()); //desired enc 
        //SmartDashboard.putString("Swerve["+absoluteEncoder.getDeviceID()+"] state", state.toString());  
    }

    //stop modules 
    public void stop(){
        drivingMotor.set(0);
        turningMotor.set(0);
    }

    //set driving motor 
    public void setDrivingMotor(double speed){
        drivingMotor.set(speed);
    }

    @Override
    public void periodic(){
        // kp = SmartDashboard.getNumber("kP", 0);
        // SmartDashboard.putNumber("kP", kp);
        // ki = SmartDashboard.getNumber("kI", 0);
        // SmartDashboard.putNumber("kI", ki);
        // kd = SmartDashboard.getNumber("kD", 0);
        // SmartDashboard.putNumber("kD", kd);

        // turningPID.setPID(kp, ki, kd);

        //SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] turning enc", getTurningPosition());
        //SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] angle", getAbsoluteEncoder());
        SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] drive speed", getDriveSpeed());
        //SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] turning speed", getTurningSpeed());
    }

}