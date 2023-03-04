package frc.robot;

import frc.robot.Constants.DriverControlConsts;
import frc.robot.commands.PitchBalance;
import frc.robot.commands.AutonomousCommands.*;
import frc.robot.commands.ClawCommands.*;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.PivotCommands.*;
import frc.robot.commands.LED_Commands.*;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.RotateLeftCommand;
import frc.robot.commands.MovementCommands.RotateRightCommand;
import frc.robot.commands.MovementCommands.RotateToAngle;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
  public static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public static PivotSubsystem pivotSubsystem = new PivotSubsystem();
  public static ClawSubsystem clawSubsystem = new ClawSubsystem();
  public static Lights lights = new Lights();

  private XboxController xbox = new XboxController(DriverControlConsts.XBOX_CONTROLLER_PORT);
  private Joystick joystick = new Joystick(DriverControlConsts.JOYSTICK_PORT);

  //AUTONOMOUS CHOICES 
  private final Command high = new High(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command hybridMiddle = new HybridMiddle(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command balance = new PitchBalance(swerveSubsystem, 0);
  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new FieldOriented(swerveSubsystem,
        () -> xbox.getLeftY() * 0.85,
        () -> xbox.getLeftX() * 0.85,
        () -> -xbox.getRightX() * 0.85)); // for field oriented drive

    lights.setDefaultCommand(new Off(lights));
    selectAuto();
    configureBindings();
  }


  private void configureBindings() {

    /* SWERVE */
    new JoystickButton(xbox, 1).toggleOnTrue(
        new DriverControl(swerveSubsystem,
            () -> -xbox.getLeftY() * 0.35,
            () -> -xbox.getLeftX() * 0.35,
            () -> -xbox.getRightX() * 0.35));
    new JoystickButton(xbox, 6).toggleOnTrue(
        new DriverControl(swerveSubsystem,
            () -> -xbox.getLeftY() * 0.75,
            () -> -xbox.getLeftX() * 0.75,
            () -> -xbox.getRightX() * 0.75));
    new JoystickButton(xbox, 2).toggleOnTrue(new Lock(swerveSubsystem)); // to lock in place :: Button B
    new JoystickButton(xbox, 4).toggleOnFalse(new Endgame(swerveSubsystem, () -> xbox.getLeftY()));

    // FOR TESTING
    new JoystickButton(xbox, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetNavx()));
    new JoystickButton(xbox, 3).onTrue(new LandingGearIn(swerveSubsystem));

    /* CLAW */
    new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));

    new JoystickButton(joystick, 8).onTrue(new Go90Clockwise(clawSubsystem));
    new JoystickButton(joystick, 10).onTrue(new ToStartingPosition(clawSubsystem));
    new JoystickButton(joystick, 12).onTrue(new Go90Counterclockwise(clawSubsystem));

    new JoystickButton(joystick, 2).whileTrue(new ManualClaw(clawSubsystem, () -> joystick.getX()));

    /* PIVOT */
    new JoystickButton(joystick, 11).onTrue(new LowPickUp(pivotSubsystem, elevatorSubsystem));
    new JoystickButton(joystick, 9)
        .onTrue(new ParallelCommandGroup(new PivotMiddleCommand(pivotSubsystem), new MidPosition(elevatorSubsystem)));
    new JoystickButton(joystick, 7).onTrue(new TopNode(pivotSubsystem, elevatorSubsystem));
    new JoystickButton(joystick, 5).onTrue(Tucked.getCommand(pivotSubsystem, elevatorSubsystem, clawSubsystem));
    new JoystickButton(joystick, 3).whileTrue(new PivotJoystickCommand(pivotSubsystem, () -> -joystick.getY()));

    /* ELEVATOR */
    new POVButton(joystick, 0).whileTrue(new ManualElevatorDrive(elevatorSubsystem, 0.5));
    new POVButton(joystick, 180).whileTrue(new ManualElevatorDrive(elevatorSubsystem, -0.5));

    /* LIGHTS */
    new JoystickButton(joystick, 6).toggleOnTrue(new Yellow(lights));
    new JoystickButton(joystick, 4).toggleOnTrue(new Violet(lights));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return new Hybrid(swerveSubsystem, clawSubsystem, pivotSubsystem,
    // elevatorSubsystem);
  }

  public void selectAuto() {
    //autoChooser.addOption("Hybrid", hybrid);
    autoChooser.addOption("High", high);
    autoChooser.addOption("Hybrid Middle", hybridMiddle);
    autoChooser.addOption("Balance", balance);

    SmartDashboard.putData(autoChooser);
  }

}
