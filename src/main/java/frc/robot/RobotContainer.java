package frc.robot;

import frc.robot.Constants.DriverControlConsts;
import frc.robot.commands.AutonomousCommands.*;
import frc.robot.commands.ClawCommands.*;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.DriveCommands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.PivotCommands.*;
import frc.robot.commands.LED_Commands.*;
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
  private Command highMobility = new HighMobility(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command highBal = new HighBal(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command high = new High(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command doNothing = new DoNothing();
  private Command redHighBalEnc = new RedHighBalEnc(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem); 
  private Command blueHighBalEnc = new BlueHighBalEnc(swerveSubsystem, clawSubsystem, pivotSubsystem, elevatorSubsystem);
  private Command mixedBalance = new MixedBalance(swerveSubsystem);
  private Command swerveLock = new Lock(swerveSubsystem);
  public SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new FieldOriented(swerveSubsystem,
        () -> xbox.getLeftY() * 0.95,
        () -> xbox.getLeftX() * 0.95,
        () -> -xbox.getRightX() * 0.95));

    selectAuto();
    configureBindings();
  }


  private void configureBindings() {

    /* SWERVE */

    // NORMAN SAID NO NEED
    /*new JoystickButton(xbox, 1).toggleOnTrue(
        new FieldOriented(swerveSubsystem,
            () -> xbox.getLeftY() * 0.35,
            () -> xbox.getLeftX() * 0.35,
            () -> -xbox.getRightX() * 0.35));
    new JoystickButton(xbox, 6).toggleOnTrue(
        new DriverControl(swerveSubsystem,
            () -> -xbox.getLeftY() * 0.75,
            () -> -xbox.getLeftX() * 0.75,
            () -> -xbox.getRightX() * 0.75));
    */
    new JoystickButton(xbox, 2).toggleOnTrue(new Lock(swerveSubsystem));
    // new JoystickButton(xbox, 4).toggleOnTrue(new Endgame(swerveSubsystem, () -> xbox.getLeftY()));
    new JoystickButton(xbox, 4).toggleOnTrue(new TankEndgame(swerveSubsystem, () -> xbox.getLeftY(), () -> -xbox.getRightY()));

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
    new JoystickButton(joystick, 9).onTrue(new ParallelCommandGroup(new PivotMiddleCommand(pivotSubsystem), new MidPosition(elevatorSubsystem)));
    new JoystickButton(joystick, 7).onTrue(new TopNode(pivotSubsystem, elevatorSubsystem));
    new JoystickButton(joystick, 5).onTrue(Tucked.getCommand(pivotSubsystem, elevatorSubsystem, clawSubsystem));
    new JoystickButton(joystick, 1).whileTrue(new PivotJoystickCommand(pivotSubsystem, () -> -joystick.getY()));

    /* ELEVATOR */
    new POVButton(joystick, 0).whileTrue(new ManualElevatorDrive(elevatorSubsystem, 0.75));
    new POVButton(joystick, 180).whileTrue(new ManualElevatorDrive(elevatorSubsystem, -0.75));

    // new JoystickButton(joystick, 3).onTrue(new LowPosition(elevatorSubsystem));

    /* LIGHTS */
    new JoystickButton(joystick, 6).toggleOnTrue(new Yellow(lights));
    new JoystickButton(joystick, 4).toggleOnTrue(new Violet(lights));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command getSwerveLock(){
    return swerveLock;
  }

  public void selectAuto() {
    autoChooser.setDefaultOption("Do Nothing", doNothing);
    autoChooser.addOption("High Mobility", highMobility);
    autoChooser.addOption("High Balance", highBal);
    autoChooser.addOption("Red High Balance", redHighBalEnc);
    autoChooser.addOption("Blue High Balance", blueHighBalEnc);
    autoChooser.addOption("High ONLY", high);
    autoChooser.addOption("Mixed Balance ONLY", mixedBalance);

    SmartDashboard.putData(autoChooser);
  }

}
