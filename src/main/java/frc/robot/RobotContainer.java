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
    // ALAINA (JOYSTICK): ELVATOR, PIVOT, CLAW, LIGHTS

    /* DRIVE */
    

    /* CLAW */
    

    /* ELEVATOR */
    

    /* PIVOT */
    


  }

  
  public Command getAutonomousCommand() {
    return null;
  }

}
