package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClawSubsystem extends SubsystemBase {
  private CANSparkMax wrist;
  private RelativeEncoder wristEnc;

  private DoubleSolenoid claw;
  
  public ClawSubsystem() {
    wrist = new CANSparkMax(15, MotorType.kBrushless);
    wrist.setIdleMode(IdleMode.kBrake);

    wristEnc = wrist.getEncoder();
    wristEnc.setPosition(0);

    claw = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 8);
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
    wrist.set(0.3);
  }

  public void counterclockwise(){
    wrist.set(-0.3);
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
    if(getEncoder()<100){
      clockwise();
    } else if(getEncoder()>100){
      counterclockwise();
    } else{
      stopWrist();
    }
  }

  public void rotateTo180(){
    if(getEncoder()<200){
      clockwise();
    } else if(getEncoder()>200){
      counterclockwise();
    } else{
      stopWrist();
    }
  }

  public void go90Clockwise(double previousEnc){
    if( getEncoder() < (previousEnc+100) ){
      clockwise();
    } else{
      stopWrist();
    }
  }

  public void go90Counterclockwise(double previousEnc){
    if( getEncoder() < (previousEnc-100) ){
      clockwise();
    } else{
      stopWrist();
    }
  }

  public void toggle(){
    claw.toggle();
  }


  @Override
  public void periodic() {
    SmartDashboard.putString("Claw", claw.get().toString());

    SmartDashboard.putNumber("Wirst Enc", wristEnc.getPosition());
  }
}