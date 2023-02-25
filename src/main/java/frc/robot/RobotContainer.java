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
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class RobotContainer {
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private ClawSubsystem clawSubsystem = new ClawSubsystem();
  private Lights lights = new Lights();

  private XboxController xbox = new XboxController(DriverControlConsts.XBOX_CONTROLLER_PORT);
  private Joystick joystick = new Joystick(DriverControlConsts.JOYSTICK_PORT);
  
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new DriverControl(swerveSubsystem, 
      () -> -xbox.getLeftY(), 
      () -> -xbox.getLeftX(), 
      () -> -xbox.getRightX(), 
      () -> xbox.getRightBumper())); // for field oriented drive
    
    configureBindings(); 
  }

  private void configureBindings() {
    // NORMAN (XBOX): SWERVE DRIVE, CLAW, ENDGAME
    // ALAINA (JOYSTICK): ELEVATOR, PIVOT, CLAW, LIGHTS

    /* DRIVE */
    new JoystickButton(xbox, 4).toggleOnTrue(new Endgame(swerveSubsystem, () -> xbox.getLeftY()));
    new JoystickButton(xbox, 2).onTrue(new Lock(swerveSubsystem));
    

    /* CLAW */
    //Norman's
    new JoystickButton(xbox, 5).onTrue(new Claw(clawSubsystem));

    //Alaina's
    
    /* ELEVATOR */
    new POVButton(joystick, 0).onTrue(new ManualElevatorDrive(elevatorSubsystem, 0.4));
    new POVButton(joystick, 180).onTrue(new ManualElevatorDrive(elevatorSubsystem, -0.2));
    new JoystickButton(joystick, 7).onTrue(new HighPosition(elevatorSubsystem));
    new JoystickButton(joystick, 11).onTrue(new ZeroPosition(elevatorSubsystem));

    /* PIVOT */
    new JoystickButton(joystick, 12).onTrue(new TuckedIn(pivotSubsystem));
    new JoystickButton(joystick, 10).onTrue(new PivotLowCommand(pivotSubsystem));
    new JoystickButton(joystick, 8).onTrue(new PivotHighCommand(pivotSubsystem));
    new JoystickButton(joystick, 3).whileTrue(new PivotArmJoystickCommand(pivotSubsystem, () -> joystick.getY()));
    

  }

  
  public Command getAutonomousCommand() {
    return null;
  }

}
