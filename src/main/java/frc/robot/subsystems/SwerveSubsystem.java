package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConsts;
import frc.robot.Constants.LandingGearConsts;
import frc.robot.Constants.SwerveConsts;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveModule frontLeft;
    private SwerveModule backLeft;
    private SwerveModule backRight;
    private SwerveModule frontRight;

    private AHRS navx;

    private DoubleSolenoid landinator;
    private CANSparkMax wheelinator;

    public SwerveSubsystem() {
        frontLeft = new SwerveModule(SwerveConsts.FL_TURN_PORT, SwerveConsts.FL_DRIVE_PORT, 
            SwerveConsts.FL_ABSOLUTE_ENCODER_PORT, SwerveConsts.FL_OFFSET , false, true, true);
        
        backLeft = new SwerveModule(SwerveConsts.BL_TURN_PORT, SwerveConsts.BL_DRIVE_PORT, 
            SwerveConsts.BL_ABSOLUTE_ENCODER_PORT, SwerveConsts.BL_OFFSET , false, true, true);

        backRight = new SwerveModule(SwerveConsts.BR_TURN_PORT, SwerveConsts.BR_DRIVE_PORT, 
            SwerveConsts.BR_ABSOLUTE_ENCODER_PORT, SwerveConsts.BR_OFFSET , false, true, true);

        frontRight = new SwerveModule(SwerveConsts.FL_TURN_PORT, SwerveConsts.FL_DRIVE_PORT, 
            SwerveConsts.FR_ABSOLUTE_ENCODER_PORT, SwerveConsts.FR_OFFSET , false, true, true);

        navx = new AHRS(SPI.Port.kMXP);


        /* * * Landing Gear * * */
        landinator = new DoubleSolenoid(PneumaticsModuleType.REVPH, LandingGearConsts.LANDING_GEAR_PISTON_FORWARD_CHANNEL, LandingGearConsts.LANDING_GEAR_PISTON_REVERSE_CHANNEL);
        wheelinator = new CANSparkMax(LandingGearConsts.LANDING_GEAR_MOTOR_PORT, MotorType.kBrushless);

        landinator.set(Value.kReverse);

    }

    public void resetNavx() {
        navx.zeroYaw();
    }

    public double getYawAngle(){
        return ( /* navx.getYaw() */  navx.getAngle() % 360  /*360-navx.getYaw()*/ );
    }

    public Rotation2d getRobotRotation(){
        return new Rotation2d(Math.toRadians(navx.getYaw()));
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getYawAngle());
    }

    public void stopModules() {
        frontLeft.stop();
        backLeft.stop();
        backRight.stop();
        frontRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConsts.MAX_SPEED);
        frontLeft.setDesiredState(desiredStates[0]);
        backLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        frontRight.setDesiredState(desiredStates[3]);
    }

    public void lock(){
        SwerveModuleState fl = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(45))); //45, -45, 45, -45
        SwerveModuleState bl = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(-45)));
        SwerveModuleState br = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(45)));
        SwerveModuleState fr = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(-45)));

        frontLeft.setAngle(fl);
        backLeft.setAngle(bl);
        backRight.setAngle(br);
        frontRight.setAngle(fr);
    }

    // PERIODIC - runs repeatedly (like periodic from timed robot)
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Yaw", getYawAngle());
    }

    public void driveForward(){
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(AutoConsts.DRIVE_TRANSLATION_SPEED, 0, 0));
        setModuleStates(moduleStates);
    }

    public void driveBackward(){
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(-AutoConsts.DRIVE_TRANSLATION_SPEED, 0, 0));
        setModuleStates(moduleStates);
    }

    public void strafeLeft(){
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, -AutoConsts.DRIVE_TRANSLATION_SPEED, 0));
        setModuleStates(moduleStates);
    }

    public void strafeRight(){
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, AutoConsts.DRIVE_TRANSLATION_SPEED, 0));
        setModuleStates(moduleStates);
    }

    public void rotateLeft(){
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, 0, -AutoConsts.DRIVE_ROTATION_SPEED));
        setModuleStates(moduleStates);
    }

    public void rotateRight(){
        SwerveModuleState[] moduleStates = SwerveConsts.DRIVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, 0, AutoConsts.DRIVE_ROTATION_SPEED));
        setModuleStates(moduleStates);
    }


    /* * * LANDING GEAR * * */

    public void wheelsIn(){
        landinator.set(Value.kReverse);
    }

    public void wheelsOut(){
        SwerveModuleState fl = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(45)));
        SwerveModuleState bl = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(-90)));
        SwerveModuleState br = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(90)));
        SwerveModuleState fr = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(-45)));

        frontLeft.setAngle(fl);
        backLeft.setAngle(bl);
        backRight.setAngle(br);
        frontRight.setAngle(fr);

        landinator.set(Value.kForward);
    }

    public void setEndgame(double speed){
        SwerveModuleState fl = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(45)));
        SwerveModuleState bl = new SwerveModuleState(speed, new Rotation2d(Math.toRadians(90)));
        SwerveModuleState br = new SwerveModuleState(speed, new Rotation2d(Math.toRadians(90)));
        SwerveModuleState fr = new SwerveModuleState(0.0, new Rotation2d(Math.toRadians(-45)));

        //wheelinator.set(speed*1.25);

        frontLeft.setAngle(fl);
        backLeft.setDesiredState(bl);
        backRight.setDesiredState(br);
        frontRight.setAngle(fr);

        wheelinator.set(-backLeft.getDriveSpeed());

    }

}
