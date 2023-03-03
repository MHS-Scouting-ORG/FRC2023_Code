package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConsts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {
  private CANSparkMax wrist;
  private RelativeEncoder wristEnc;

  private DoubleSolenoid claw;
  
  public ClawSubsystem() {
    wrist = new CANSparkMax(ClawConsts.WRIST_MOTOR_PORT, MotorType.kBrushless);
    wrist.setIdleMode(IdleMode.kBrake);

    wristEnc = wrist.getEncoder();
    wristEnc.setPosition(0);

    //  forward=open    reverse=closed
    claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClawConsts.CLAW_FORWARD_CHANNEL, ClawConsts.CLAW_REVERSE_CHANNEL);
    claw.set(Value.kForward);
  }

  public void resetEnc(){
    wristEnc.setPosition(0);
  }

  public double getEncoder(){
    return wristEnc.getPosition();

  }

  public void manualRotate(double speed){
    wrist.set(speed);
  }

  public void clockwise(){
    wrist.set(ClawConsts.WRIST_SPEED);
  }

  public void counterclockwise(){
    wrist.set(-ClawConsts.WRIST_SPEED);
  }

  public void stopWrist(){
    wrist.set(0);
  }

  public void startingPosition(){
    if(getEncoder()<0){
      clockwise();
    } else if(getEncoder()>0){
      counterclockwise();
    } else{
      stopWrist();
    }
  }

  public void rotateTo90(){
    if(getEncoder()<ClawConsts.ROTATION_T0_90_ENC){
      clockwise();
    } else if(getEncoder()>ClawConsts.ROTATION_T0_90_ENC){
      counterclockwise();
    } else{
      stopWrist();
    }
  }

  public void rotateTo180(){
    if(getEncoder()<ClawConsts.ROTATION_TO_180_ENC){
      clockwise();
    } else if(getEncoder()>ClawConsts.ROTATION_TO_180_ENC){
      counterclockwise();
    } else{
      stopWrist();
    }
  }

  public void go90Clockwise(double previousEnc){
    if( getEncoder() < (previousEnc+ClawConsts.ROTATE_90) ){
      clockwise();
    } else{
      stopWrist();
    }
  }

  public void go90Counterclockwise(double previousEnc){
    if( getEncoder() < (previousEnc-ClawConsts.ROTATE_90) ){
      counterclockwise();
    } else{
      stopWrist();
    }
  }

  public void openClaw(){
    claw.set(Value.kReverse);
  }

  public void closeClaw(){
    claw.set(Value.kForward);
  }

  public void toggle(){
    claw.toggle();
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Closed?", claw.get().toString().equals("kForward"));

    SmartDashboard.putNumber("Wrist Enc", wristEnc.getPosition());
  }
}