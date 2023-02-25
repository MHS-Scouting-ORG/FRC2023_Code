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

    public SwerveModule(int turnPort, int drivePort, int cancoderPort, double encoderOffset, boolean encoderReversed, boolean driveReversed, boolean turnReversed){
        turningMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
        drivingMotor = new CANSparkMax(drivePort, MotorType.kBrushless);

        absoluteEncoder = new CANCoder(cancoderPort);
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        turningEnc = turningMotor.getEncoder();
        drivingEnc = drivingMotor.getEncoder();

        turningPID = new PIDController(SwerveConsts.KP_TURNING, SwerveConsts.KI_TURNING, SwerveConsts.KD_TURNING);
        turningPID.enableContinuousInput(-Math.PI, Math.PI); // System is circular;  Goes from -Math.PI to 0 to Math.PI

        // drivingPID = new PIDController(SwerveConsts.kp_driving, SwerveConsts.ki_driving, SwerveConsts.kd_driving);

        this.encoderOffset = encoderOffset;
        this.encoderReversed = encoderReversed;

        turningEnc.setPositionConversionFactor(SwerveConsts.TURNING_ENCODER_ROTATION_CONVERSION);
        turningEnc.setVelocityConversionFactor(SwerveConsts.TURNING_ENCODER_SPEED_CONVERSION);

        drivingMotor.setInverted(driveReversed);
        turningMotor.setInverted(turnReversed);

        drivingMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        resetEncoders();

    }

    /* * * ENCODER VALUES * * */

    public double getDrivePosition(){
        //return (drivingEnc.getPosition() % 900) * 360;
        return drivingEnc.getPosition();
    }

    // neo encoder in degrees 
    public double getTurningPosition(){
        //return (turningEnc.getPosition() % 900) / 900 * 360;
        return turningEnc.getPosition();
    }

    public double getDriveSpeed(){
        //return drivingEnc.getVelocity() / 60 * SwerveConsts.wheelDiameter * Math.PI;
        return drivingMotor.get();
    }

    public double getTurningSpeed(){
        //return turningEnc.getVelocity() / 60 * 2 * Math.PI;
        return turningEnc.getVelocity();
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

    // set turning enc to value of absolute encoder
    public void resetEncoders(){
        drivingEnc.setPosition(0);
        turningEnc.setPosition(getAbsoluteEncoder());
    }

    public SwerveModuleState getState(){
        // Rotation2d = rotation represented by a point on the unit circle
        // Rotation2d(double) => constructs a Rotation2d given the angle in radians
        
        // SwerveModuleState = state of a swerve module
        // SwerveModuleState(speed (in meters per second), angle of module (Using Rotation2d))
        return new SwerveModuleState(getDriveSpeed(), new Rotation2d(getAbsoluteEncoder()));
    }

    public void setDesiredState(SwerveModuleState state){
        // To make keep robot from going back to 0 position
        if(Math.abs(state.speedMetersPerSecond) < 0.1){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);

        // set speed
        drivingMotor.set(state.speedMetersPerSecond / SwerveConsts.MAX_SPEED * SwerveConsts.VOLTAGE); 
        // drivingMotor.set(drivingPID.calculate((drivingMotor.get() * SwerveConsts.maxSpeed_mps), state.speedMetersPerSecond) * SwerveConsts.voltage);
        turningMotor.set(turningPID.calculate(getAbsoluteEncoder(), state.angle.getRadians()));

        // Print to SmartDashboard
        //SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] desired enc", state.angle.getRadians()); //desired enc 
        SmartDashboard.putString("Swerve["+absoluteEncoder.getDeviceID()+"] state", state.toString());
        SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] drive speed", getDriveSpeed());
        SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] angle", getAbsoluteEncoder());
    }

    public void setAngle(SwerveModuleState state){

        state = SwerveModuleState.optimize(state, getState().angle);

        // set speed
        drivingMotor.set(0); 
        turningMotor.set(turningPID.calculate(getAbsoluteEncoder(), state.angle.getRadians()));

        // Print to SmartDashboard
        SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] desired enc", state.angle.getRadians()); //desired enc 
        SmartDashboard.putString("Swerve["+absoluteEncoder.getDeviceID()+"] state", state.toString());  
    }

    public void stop(){
        drivingMotor.set(0);
        turningMotor.set(0);
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

        SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] abs deg enc", Math.toDegrees(getAbsoluteEncoder()));
        SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] angle", getAbsoluteEncoder());
        SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] drive speed", getDriveSpeed());
        SmartDashboard.putNumber("Swerve["+absoluteEncoder.getDeviceID()+"] turning speed", getTurningSpeed());
    }

}