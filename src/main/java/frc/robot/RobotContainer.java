package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Claw;
import frc.robot.commands.To180Position;
import frc.robot.commands.To90Position;
import frc.robot.commands.ToStartingPosition;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private ClawSubsystem clawSubsystem = new ClawSubsystem();
  private Lights lights = new Lights();

  private XboxController xbox = new XboxController(0);
  
  public RobotContainer() {
    //clawSubsystem.setDefaultCommand(new InstantCommand(()->clawSubsystem.manualRotate(xbox.getLeftY())));
    //clawSubsystem.setDefaultCommand(new InstantCommand(()->clawSubsystem.manualRotate(0)));
    configureBindings(); 
  }

  private void configureBindings() {
    new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));                // open/close claw
    new JoystickButton(xbox, 4).onTrue(new ToStartingPosition(clawSubsystem));  // starting position
    new JoystickButton(xbox, 3).onTrue(new To90Position(clawSubsystem));        // to 90 position
    new JoystickButton(xbox, 1).onTrue(new To180Position(clawSubsystem));       // to 180 position

    //new JoystickButton(xbox, 6).whileTrue(new InstantCommand(()->clawSubsystem.clockwise()));
    //new JoystickButton(xbox, 2).whileTrue(new InstantCommand(()->clawSubsystem.counterclockwise()));
  }

  
  public Command getAutonomousCommand() {
    return null;
  }
}
