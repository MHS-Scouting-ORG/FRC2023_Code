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
  private Joystick joystick = new Joystick(DriverControlConsts.JOYSTICK_PORT);

  //private XboxController testController = new XboxController(2);
  
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
    new JoystickButton(xbox, 2).toggleOnTrue(new Lock(swerveSubsystem)); // to lock in place :: Button B
    new JoystickButton(xbox, 4).toggleOnFalse(new Endgame(swerveSubsystem, () -> xbox.getLeftY())); // to deploy endgame
    /* !!! TEST !!! */ new JoystickButton(xbox, 1).whileTrue(new Rotatinate(swerveSubsystem, () -> xbox.getRightX(),  () -> xbox.getRightY()));
    
    // FOR TESTING
    new JoystickButton(xbox, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetNavx()));
    new JoystickButton(xbox, 3).onTrue(new LandingGearIn(swerveSubsystem));

    /* CLAW */
    new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));

    new JoystickButton(joystick, 8).onTrue(new ToStartingPosition(clawSubsystem));
    new JoystickButton(joystick, 10).onTrue(new To90Position(clawSubsystem));
    new JoystickButton(joystick, 12).onTrue(new To180Position(clawSubsystem));


    /* PIVOT */
    new JoystickButton(joystick, 11).onTrue(new LowPickUp(pivotSubsystem, elevatorSubsystem));
    // Middle
    new JoystickButton(joystick, 7).onTrue(new TopNode(pivotSubsystem, elevatorSubsystem));
    new JoystickButton(joystick, 5).onTrue(new ProxyCommand(() -> {
      if (elevatorSubsystem.getEncoder() < 160) {
        return new TuckedFromBottom(pivotSubsystem, elevatorSubsystem);
      } else {
        return new TuckedFromTop(pivotSubsystem, elevatorSubsystem);
      }}
    ));

    // MANUAL
    new JoystickButton(joystick, 2).whileTrue(new PivotJoystickCommand(pivotSubsystem, ()->joystick.getY()));

    /* MANUAL ELEVATOR */
    new POVButton(joystick, 0).whileTrue(new ManualElevatorDrive(elevatorSubsystem, 0.5));
    new POVButton(joystick, 180).whileTrue(new ManualElevatorDrive(elevatorSubsystem, -0.5));

    /* LIGHTS */
    new JoystickButton(joystick, 6).toggleOnTrue(new Yellow(lights));
    new JoystickButton(joystick, 4).toggleOnTrue(new Violet(lights));

  }

  
  public Command getAutonomousCommand() {
    return null;
  }

}
