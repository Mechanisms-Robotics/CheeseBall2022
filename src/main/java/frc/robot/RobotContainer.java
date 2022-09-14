package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StopIntakingCommand;
import frc.robot.commands.StopShootingCommand;
import frc.robot.commands.swerve.DriveTeleopCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ControllerWrapper;

/** This class contains all the subsystems, command bindings, and button bindings */
public class RobotContainer {
  // Subsystems
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();
  private final Processor processor = new Processor();
  private final Feeder feeder = new Feeder();

  // Superstructure
  private final Superstructure superstructure = new Superstructure(intake, processor, feeder);

  // Controller
  private final ControllerWrapper driverController = new ControllerWrapper(0);

  // Buttons
  private final Button gyroResetButton = new Button(driverController::getShareButton);
  private final Button toggleIntakeButton = new Button(driverController::getLeftTriggerButton);
  private final Button toggleShootButton = new Button(driverController::getRightTriggerButton);

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
  private void configureButtonBindings() {
    // When the gyroResetButton is pressed, re-zero the swerve heading
    gyroResetButton.whenPressed(new InstantCommand(swerve::zeroHeading));

    // When the toggleIntakeButton is pressed, either run the IntakeCommand or StopIntakingCommand
    // depending on whether the superstructure is currently intaking
    toggleIntakeButton.whenPressed(
        new ConditionalCommand(
            new StopIntakingCommand(superstructure),
            new IntakeCommand(superstructure),
            superstructure::isIntaking));

    // When the toggleShootButton is pressed, either run the ShootCommand or StopShootingCommand
    // depending on whether the superstructure is currently shooting
    toggleShootButton.whenPressed(
        new ConditionalCommand(
            new StopShootingCommand(superstructure),
            new ShootCommand(superstructure),
            superstructure::isShooting));
  }

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
