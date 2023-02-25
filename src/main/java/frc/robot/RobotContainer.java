package frc.robot;

import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.ClawCommands.To180Position;
import frc.robot.commands.ClawCommands.To90Position;
import frc.robot.commands.ClawCommands.ToStartingPosition;
import frc.robot.commands.LED_Commands.Violet;
import frc.robot.commands.LED_Commands.Yellow;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private ClawSubsystem clawSubsystem = new ClawSubsystem();
  private Lights lights = new Lights();

  private XboxController xbox = new XboxController(0);
  
  public RobotContainer() {
    
    configureBindings(); 
  }

  private void configureBindings() {
    new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));                // open/close claw
    new JoystickButton(xbox, 4).onTrue(new ToStartingPosition(clawSubsystem));  // starting position
    new JoystickButton(xbox, 2).onTrue(new To90Position(clawSubsystem));        // to 90 position
    new JoystickButton(xbox, 1).onTrue(new To180Position(clawSubsystem));       // to 180 position

    new JoystickButton(xbox, 6).whileTrue(new Yellow(lights));
    new JoystickButton(xbox, 7).whileTrue(new Violet(lights));

  }

  
  public Command getAutonomousCommand() {
    return null;
  }
}
