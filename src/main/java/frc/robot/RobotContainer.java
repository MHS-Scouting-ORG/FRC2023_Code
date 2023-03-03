package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriverControl;
import frc.robot.commands.IncrementBalanceCommand;
import frc.robot.commands.PitchBalance;
import frc.robot.commands.ArmCommands.Tucked;
import frc.robot.commands.AutonomousCommands.High;
import frc.robot.commands.AutonomousCommands.Hybrid;
import frc.robot.commands.ClawCommands.Claw;
import frc.robot.commands.MovementCommands.DriveBackwardCommand;
import frc.robot.commands.MovementCommands.DriveForwardCommand;
import frc.robot.commands.MovementCommands.RotateRightCommand;
import frc.robot.commands.MovementCommands.StrafeRightCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriverControl;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {

  //INIT OBJECTS
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ClawSubsystem claw = new ClawSubsystem(); 
  private final ElevatorSubsystem elev = new ElevatorSubsystem(); 
  private final PivotSubsystem pivot = new PivotSubsystem();
  private final Timer timer = new Timer(); 

  private final XboxController m_Controller = new XboxController(0); 

  public RobotContainer() {
    /* * * Driver Control Default * * */
    swerve.setDefaultCommand(new DriverControl(swerve, 
    () -> -m_Controller.getLeftY(), 
    () -> -m_Controller.getLeftX(),
    () -> -m_Controller.getRightX(), 
    () -> m_Controller.getRightBumper()));

    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(m_Controller, 1).onTrue(new Claw(claw)); 
    new JoystickButton(m_Controller, 2).onTrue(new InstantCommand(() -> swerve.resetNavx())); 
    new JoystickButton(m_Controller, 3).onTrue(Tucked.getCommand(pivot, elev));
    new JoystickButton(m_Controller, 4).onTrue(new DriveForwardCommand(swerve, 27));
  }

  public Command getAutonomousCommand() {
    timer.reset();
    timer.start();
    return new Hybrid(swerve, claw, pivot, elev, timer.get());
  }
}
