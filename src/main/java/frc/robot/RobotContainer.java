package frc.robot;

import frc.robot.Constants.DriverControlConsts;
import frc.robot.commands.ClawCommands.*;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.PivotCommands.*;
import frc.robot.commands.LED_Commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private ClawSubsystem clawSubsystem = new ClawSubsystem();
  private Lights lights = new Lights();

  private XboxController xbox = new XboxController(DriverControlConsts.XBOX_CONTROLLER_PORT);
  //private XboxController testController = new XboxController(1);
  //private Joystick joystick = new Joystick(DriverControlConsts.JOYSTICK_PORT);
  
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new DriverControl(swerveSubsystem, 
      () -> -xbox.getLeftY(), 
      () -> -xbox.getLeftX(), 
      () -> -xbox.getRightX(), 
      () -> xbox.getRightBumper())); // for field oriented drive
    
    configureBindings(); 
  }

  private void configureBindings() {

    /* SWERVE */
    // new JoystickButton(xbox, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetNavx()));
    // new JoystickButton(xbox, 2).toggleOnTrue(new Lock(swerveSubsystem)); // to lock in place :: Button B
    // new JoystickButton(xbox, 4).toggleOnFalse(new Endgame(swerveSubsystem, () -> xbox.getLeftY())); // to deploy endgame
    // new JoystickButton(xbox, 1).onTrue(new LandingGearIn(swerveSubsystem));

    // /* !!! TEST !!! */ new JoystickButton(xbox, 3).whileTrue(new Rotatinate(swerveSubsystem, () -> xbox.getRightX(),  () -> xbox.getRightY()));

    /* CLAW */
    new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));
    // new POVButton(xbox, 0).onTrue(new ToStartingPosition(clawSubsystem));
    // new POVButton(xbox, 270).onTrue(new To90Position(clawSubsystem));
    // new POVButton(xbox, 180).onTrue(new To180Position(clawSubsystem));


    /* PIVOT */
    new JoystickButton(xbox, 1).onTrue(new LowPickUp(pivotSubsystem, elevatorSubsystem));
    new JoystickButton(xbox, 4).onTrue(new TopNode(pivotSubsystem, elevatorSubsystem));
    new JoystickButton(xbox, 3).onTrue(new ProxyCommand(() -> {
      if (elevatorSubsystem.getEncoder() < 160) {
        return new TuckedFromBottom(pivotSubsystem, elevatorSubsystem);
      } else {
        return new TuckedFromTop(pivotSubsystem, elevatorSubsystem);
      }}
    ));
  }

  
  public Command getAutonomousCommand() {
    return null;
  }

}
