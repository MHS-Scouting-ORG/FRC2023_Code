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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

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

    /* SWERVE */
    new JoystickButton(xbox, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetNavx()));
    new JoystickButton(xbox, 2).toggleOnTrue(new Lock(swerveSubsystem)); // to lock in place :: Button B
    new JoystickButton(xbox, 4).toggleOnFalse(new Endgame(swerveSubsystem, () -> xbox.getLeftY())); // to deploy endgame
    new JoystickButton(xbox, 1).onTrue(new LandingGearIn(swerveSubsystem));

    /* !!! TEST !!! */ new JoystickButton(xbox, 3).whileTrue(new Rotatinate(swerveSubsystem, () -> xbox.getRightX(),  () -> xbox.getRightY()));

    /* CLAW */
    new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));
    // new JoystickButton(xbox, xbox.getPOV(270)).onTrue(new Go90Counterclockwise(clawSubsystem));
    // new JoystickButton(xbox, xbox.getPOV(90)).onTrue(new Go90Clockwise(clawSubsystem));

    /* ELEVATOR */
    new JoystickButton(testController, 4).onTrue(new HighPosition(elevatorSubsystem)); // high position
    //new JoystickButton(testController, 4).onTrue(new MidPosition(elevatorSubsystem)); // mid position
    new JoystickButton(testController, 7).onTrue(new LowPosition(elevatorSubsystem)); // low position
    new JoystickButton(testController, 1).onTrue(new ZeroPosition(elevatorSubsystem)); // starting position

    //new JoystickButton(testController, 5).whileTrue(new Test(elevatorSubsystem, () -> testController.getRightY()));
    // new JoystickButton(testController, testController.getPOV(180)).whileTrue(new ManualElevatorDrive(elevatorSubsystem, -0.3));
    // new JoystickButton(testController, testController.getPOV(0)).whileTrue(new ManualElevatorDrive(elevatorSubsystem, 0.3));


    /* PIVOT */
    new JoystickButton(testController, 2).onTrue(new PivotHighCommand(pivotSubsystem));
    new JoystickButton(testController, 3).onTrue(new PivotLowCommand(pivotSubsystem));
    new JoystickButton(testController, 6).onTrue(new TuckedIn(pivotSubsystem));

    new JoystickButton(testController, 5).whileTrue(new PivotArmJoystickCommand(pivotSubsystem, () -> testController.getLeftX()));

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
