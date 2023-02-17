package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  public final XboxController m_Controller = new XboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
  
  }


  public Command getAutonomousCommand() {
    return null;
  }
}
