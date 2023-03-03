package frc.robot;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonEncoder extends SubsystemBase {

private TalonSRX srx;

public TalonEncoder(TalonSRX srx) {
if(srx == null)
throw new RuntimeException("You didn't pass a value in!!!!!!!!!!!!!!!!!!!");
this.srx = srx;

}
public int get() {
return srx.getSensorCollection().getQuadraturePosition();
}

public void reset() {
srx.getSensorCollection().setQuadraturePosition(0, 5);

}

public double getPeriod() {
throw new RuntimeException();
}

public void setMaxPeriod(double maxPeriod) {
throw new RuntimeException();

}


public boolean getStopped() {
throw new RuntimeException();
}


public boolean getDirection() {
throw new RuntimeException();

}

}