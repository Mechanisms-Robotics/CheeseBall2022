package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swerve.DriveTeleopCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ControllerWrapper;

/** This class contains all the subsystems, command bindings, and button bindings */
public class RobotContainer {
  // Subsystems
  private final Swerve swerve = new Swerve();

  // Controller
  private final ControllerWrapper driverController = new ControllerWrapper(0);

  /** Constructor for the RobotContainer class */
  public RobotContainer() {
    // Zero the heading of the swerve
    swerve.zeroHeading();

    // Configure the button bindings
    configureButtonBindings();

    // Configure the default commands
    configureDefaultCommands();
  }

  /** Configures all the button bindings */
  private void configureButtonBindings() {}

  /** Configures the default commands for each subsystem */
  private void configureDefaultCommands() {
    // Set the default swerve command to a DriveTeleopCommand
    swerve.setDefaultCommand(
        new DriveTeleopCommand(
            driverController::getLeftJoystickX,
            driverController::getLeftJoystickY,
            driverController::getRightJoystickX,
            swerve));
  }

  /** Returns the command to run during autonomous */
  public Command getAutonomousCommand() {
    return null;
  }
}
