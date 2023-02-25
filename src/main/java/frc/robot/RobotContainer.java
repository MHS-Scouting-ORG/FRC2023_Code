package frc.robot;

import frc.robot.Constants.DriverControlConsts;
import frc.robot.commands.ClawCommands.*;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.PivotCommands.*;
import frc.robot.commands.SequentialElevatorPivotCommands.*;
import frc.robot.commands.LED_Commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private ClawSubsystem clawSubsystem = new ClawSubsystem();
  private Lights lights = new Lights();

  private XboxController xbox = new XboxController(DriverControlConsts.XBOX_CONTROLLER_PORT);
  private XboxController testController = new XboxController(1);
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
    // CURRENT CONFIG : 
    // XBOX 1: SWERVE DRIVE, CLAW, WRIST, ENDGAME
    // WHEN DONE WITH CLAW | XBOX 1: SWERVE DRIVE, ENDGAME, PIVOT
    // XBOX 2: PIVOT, ELEVATOR

    /* SWERVE */
    new JoystickButton(xbox, 3).toggleOnTrue(new Lock(swerveSubsystem)); // to lock in place :: Button Y
    new JoystickButton(xbox, 4).toggleOnFalse(new Endgame(swerveSubsystem, () -> xbox.getLeftY())); // to deploy endgame
    /* !!! TEST !!! */ new JoystickButton(xbox, 7).whileTrue(new Rotatinate(swerveSubsystem, () -> xbox.getRightX(),  () -> xbox.getRightY()));

    /* CLAW */
    new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));
    new JoystickButton(xbox, 1).onTrue(new Go90Clockwise(clawSubsystem));
    new JoystickButton(xbox, 2).onTrue(new Go90Counterclockwise(clawSubsystem));

    /* ELEVATOR */
    new JoystickButton(testController, 4).onTrue(new HighPosition(elevatorSubsystem)); // high position
    new JoystickButton(testController, 3).onTrue(new MidPosition(elevatorSubsystem)); // mid position
    new JoystickButton(testController, 1).onTrue(new LowPosition(elevatorSubsystem)); // low position

    /* PIVOT */
    new JoystickButton(testController, 0);

    /* * * PIVOT ROBOT CONTAINER * * *//* 
    new JoystickButton(xbox, 1).onTrue(new PivotHighCommand(pivotSubsystem));
    // new JoystickButton(xbox, 2).onTrue(tucked); // Button for the starting position
    new JoystickButton(xbox, 3).onTrue(new LowPosition(elevatorSubsystem)); // Button for the middle position
    new JoystickButton(xbox, 4).onTrue(new PivotLowCommand(pivotSubsystem)); // Button for the high position
    new JoystickButton(xbox, 5).onTrue(new MidPosition(elevatorSubsystem)); 
    new JoystickButton(xbox, 6).onTrue(new ZeroPosition(elevatorSubsystem)); // Button for driving the motor using the joystick
    new JoystickButton(xbox, 7).whileTrue(new PivotArmButtonCommand(pivotSubsystem, .3));
    new JoystickButton(xbox, 8).whileTrue(new PivotArmButtonCommand(pivotSubsystem, -.3));*/

  }

  
  public Command getAutonomousCommand() {
    return null;
  }

  /* * * NORMAN CONTROLS * * *\/
   *
   * 
   * 
   * 
   */
}
