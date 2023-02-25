package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  
  private AddressableLED lights;
  private AddressableLEDBuffer buff;

  public Lights() {
    lights = new AddressableLED(0);

    buff = new AddressableLEDBuffer(60);

    lights.setLength(buff.getLength());
    lights.setData(buff);

    lights.start();

  }

  public void off(){
    //lights.stop();
    for(int i=0; i<buff.getLength(); i++){
      buff.setRGB(i, 0, 0, 0);
    }
    lights.setData(buff);
  }

  public void sunflower(){
    for(int i=0; i<buff.getLength(); i++){
      buff.setRGB(i, 255, 255, 0);
    }
    lights.setData(buff);
  }

  public void lavendar(){
    for(int i=0; i<buff.getLength(); i++){
      buff.setRGB(i, 255, 0, 255);
    }
    lights.setData(buff);
  }

  public void potOfGold(){
    for(int i=0; i<buff.getLength(); i+=6){
      buff.setRGB(i, 255, 0, 0);
    }
    for(int i=1; i<buff.getLength(); i+=6){
      buff.setRGB(i, 255, 165, 0);
    }
    for(int i=2; i<buff.getLength(); i+=6){
      buff.setRGB(i, 255, 255, 0);
    }
    for(int i=3; i<buff.getLength(); i+=6){
      buff.setRGB(i, 0, 255, 0);
    }
    for(int i=4; i<buff.getLength(); i+=6){
      buff.setRGB(i, 255, 0, 255);
    }
    for(int i=5; i<buff.getLength(); i+=6){
      buff.setRGB(i, 255, 0, 255);
    }
    lights.setData(buff);
  }

  @Override
  public void periodic() {
    
  }
}
