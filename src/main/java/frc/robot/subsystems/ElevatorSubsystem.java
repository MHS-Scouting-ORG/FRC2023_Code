package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConsts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;

public class ElevatorSubsystem extends SubsystemBase {
  boolean pidOn = true;
  CANSparkMax elevator;
  RelativeEncoder enc;
  DigitalInput topLim;
  DigitalInput bottomLim;
  PIDController pid = new PIDController(0.1, 0, 0);
  double setpoint = 0;
  double lastError = 0;
  double manualSpeed = 0;

  public ElevatorSubsystem() {
    topLim = new DigitalInput(ElevatorConsts.TOP_LIMIT_SWITCH);
    bottomLim = new DigitalInput(ElevatorConsts.BOTTOM_LIMIT_SWITCH);
    elevator = new CANSparkMax(ElevatorConsts.ELEVATOR_ID, MotorType.kBrushless);
    enc = elevator.getEncoder();
    setpoint = enc.getPosition();
    pid.setTolerance(.05);
  }

  public void zeroEncoder(){
    
  }

  public double getEncoder(){
    return enc.getPosition();
  }

  public boolean isAtSetpoint(){ // RETURNS TRUE IF IT IS AT THE SETPOINT
    double error = setpoint - enc.getPosition();
    return Math.abs(error) < 2;
  }

  public void setManualSpeed(double inputSpeed){
    manualSpeed = inputSpeed; // WHERE MANUAL SPEED IS SET AND CHANGED
  }

  public void changeSetpoint(double setpoint) { // CHANGES DESIRED SETPOINT OF THE ELEVATOR
    this.setpoint = setpoint;
  }

  public void init(){ 
    elevator.setIdleMode(IdleMode.kBrake); // SETS ELEVATOR MOTOR INTO BRAKE MODE
  }

  public boolean topPressed(){ // RETURNS TRUE IF TOP LIMIT SWITCH IS PRESSED
    return !topLim.get();
  }

  public boolean bottomPressed(){ // RETURNS TRUE IF BOTTOM LIMIT SWITCH IS PRESSED
    return !bottomLim.get();
  }

  public void enablePid(){ // ENABLES PID -> PID WILL RUN AT ALL TIMES WHEN THIS IS TRUE
    pidOn = true; 
  }

  public void disablePid(){ // DISABLES PID -> ALLOWS FOR MANUAL DRIVE
    pidOn = false;
  }

  public void currentEncValtoSetpoint(){ // GETS THE CURRENT ENCODER VALUE
    setpoint = enc.getPosition();
  }

  public void pidIValue(double encoderPos){ // ADJUSTS AND CONTROLS THE PID I VALUE
    double currentError = setpoint - encoderPos;
    if(currentError > 0 && lastError < 0){ 
      pid.reset();
    }
    else if(currentError < 0 && lastError > 0){
      pid.reset();
    }
  }

  /* 
  public void limitStop(double speed){ // WHEN THE LIMIT SWITCH IS PRESSED, STOPS THE MOTORS FROM RUNNING
    if(topPressed()){
      speed = 0;
    }
    else if(bottomPressed()){ // -9 is the encoder count < - low position
      speed = 0;
    }
  } */

  public double getError(){
    return pid.getPositionError();
  }

  @Override
  public void periodic() {
    double encoderValue = enc.getPosition(); // GETS CURRENT ENC VALUE OF ELEVATOR
    pidIValue(encoderValue); // ADJUSTS I VALUE IN PID
    
    double calcSpeed = 0;
    if(pidOn){ // IF PID IS ENABLED, SETS THE ELEVATOR SPEED TO PID SPEED
      calcSpeed = pid.calculate(encoderValue, setpoint); 
    }
    else{
      calcSpeed = manualSpeed; // IF PID DISABLED, SETS ELEVATOR SPEED TO SPEED FOR MANUAL DRIVE
    }
    
    if(calcSpeed > 0.9){ // IF SPEED CALCULATED IS GREATER THAN 1, SETS MAX SPEED TO 1
      calcSpeed = 0.9;
    }
    else if(calcSpeed < -0.6){ // IF SPEED CALCULATED IS LESS THAN -1, SETS MAX SPEED TO -1
      calcSpeed = -0.6; 
    }

    if(topPressed() && calcSpeed > 0){
      calcSpeed = 0;
    }
    else if(bottomPressed() && calcSpeed < 0){
      calcSpeed = 0;
    }
    
    elevator.set(calcSpeed);
    //SmartDashboard.putNumber("PID Speed", calcSpeed);
    //SmartDashboard.putBoolean("Top switch pressed" , topPressed()); 
    //SmartDashboard.putBoolean("Bottom switch pressed", bottomPressed());
    SmartDashboard.putNumber("encoder counts", encoderValue);
    //SmartDashboard.putNumber("Setpoint", setpoint);
    //SmartDashboard.putNumber("error", getError());
    //SmartDashboard.putNumber("tolerance", pid.getPositionTolerance());  
  }
}