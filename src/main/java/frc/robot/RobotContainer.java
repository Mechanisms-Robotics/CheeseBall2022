package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.AimHoodCommand;
import frc.robot.commands.AimShooterCommand;
import frc.robot.commands.AimTurretCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PoseEstimateCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SmartShootCommand;
import frc.robot.commands.StopIntakingCommand;
import frc.robot.commands.StopShootingCommand;
import frc.robot.commands.StopSmartShootingCommand;
import frc.robot.commands.auto.FiveBallAutoCommand;
import frc.robot.commands.auto.FiveBallMovingAutoCommand;
import frc.robot.commands.auto.SixBallAutoCommand;
import frc.robot.commands.auto.SixBallMovingAutoCommand;
import frc.robot.commands.swerve.DriveTeleopCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.GoalTracker;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.util.ControllerWrapper;

/** This class contains all the subsystems, command bindings, and button bindings */
public class RobotContainer {
  // Subsystems
  private final Swerve swerve = new Swerve();
  private final Intake intake = new Intake();
  private final Processor processor = new Processor();
  private final Feeder feeder = new Feeder();
  public final Turret turret = new Turret();
  private final Shooter shooter = new Shooter();
  public final Hood hood = new Hood();

  // Goal Tracker
  public final GoalTracker goalTracker = new GoalTracker();

  // Superstructure
  private final Superstructure superstructure =
      new Superstructure(intake, processor, feeder, swerve::getAcceleration, swerve::getPose);

  // Controller
  private final ControllerWrapper driverController = new ControllerWrapper(0);

  // Buttons
  private final Button gyroResetButton = new Button(driverController::getShareButton);
  private final Button toggleIntakeButton = new Button(driverController::getLeftTriggerButton);
  private final Button shootButton = new Button(driverController::getRightTriggerButton);
  private final Button toggleSmartShootButton = new Button(driverController::getRightBumperButton);

  // Autos Enumerator
  private enum Autos {
    FIVE_BALL,
    FIVE_BALL_MOVING,
    SIX_BALL,
    SIX_BALL_MOVING
  }

  // Auto Chooser
  private final SendableChooser<Autos> autoChooser = new SendableChooser<>();

  /** Constructor for the RobotContainer class */
  public RobotContainer() {
    // Zero the heading of the swerve
    swerve.zeroHeading();

    // Configure the button bindings
    configureButtonBindings();

    // Configure the default commands
    configureDefaultCommands();

    // Add autonomous commands to auto chooser
    autoChooser.addOption(Autos.FIVE_BALL.name(), Autos.FIVE_BALL);
    autoChooser.addOption(Autos.FIVE_BALL_MOVING.name(), Autos.FIVE_BALL_MOVING);
    autoChooser.addOption(Autos.SIX_BALL.name(), Autos.SIX_BALL);
    autoChooser.addOption(Autos.SIX_BALL_MOVING.name(), Autos.SIX_BALL_MOVING);

    // Set default auto chooser option
    autoChooser.setDefaultOption(Autos.FIVE_BALL.name(), Autos.FIVE_BALL);

    // Put the auto chooser on the SmartDashboard
    SmartDashboard.putData(autoChooser);
  }

  /** Configures all the button bindings */
  private void configureButtonBindings() {
    // When the gyroResetButton is pressed, re-zero the swerve heading
    gyroResetButton.whenPressed(new InstantCommand(swerve::zeroHeading));

    // When the toggleIntakeButton is pressed, either run an IntakeCommand or StopIntakingCommand
    // depending on whether the superstructure is currently intaking
    toggleIntakeButton.whenPressed(
        new ConditionalCommand(
            new StopIntakingCommand(superstructure),
            new IntakeCommand(superstructure),
            superstructure::isIntaking));

    // When the shoot button is held run a ShootCommand
    shootButton.whenHeld(new ShootCommand(superstructure));

    // When the shoot button is released run a StopShootingCommand
    shootButton.whenReleased(new StopShootingCommand(superstructure));

    // When the toggle smart shoot button is pressed, either run a SmartShootCommand or a
    // StopSmartShootingCommand depending on whether the superstructure is currently smart shooting
    toggleSmartShootButton.whenPressed(
        new ConditionalCommand(
            new StopSmartShootingCommand(superstructure),
            new SmartShootCommand(superstructure),
            superstructure::isSmartShooting));
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

    // Set the default goal tracker command to a PoseEstimateCommand
    goalTracker.setDefaultCommand(
        new PoseEstimateCommand(
            goalTracker, swerve.poseEstimator, turret::getAngle, swerve::getHeading));

    // Set the default turret command to an AimTurretCommand
    turret.setDefaultCommand(
        new AimTurretCommand(turret, swerve::getPose, swerve::getSpeeds, swerve::getHeading));

    // Set the default shooter command to an AimShooterCommand
    shooter.setDefaultCommand(
        new AimShooterCommand(shooter, swerve::getPose, swerve::getSpeeds, swerve::getHeading));

    // Set the default hood command to an AimHoodCommand
    hood.setDefaultCommand(
        new AimHoodCommand(hood, swerve::getPose, swerve::getSpeeds, swerve::getHeading));
  }

  /** Returns the command to run during autonomous */
  public Command getAutonomousCommand() {
    // Check which auto is selected and return the corresponding command
    switch (autoChooser.getSelected()) {
      case FIVE_BALL:
        return new FiveBallAutoCommand(swerve, superstructure);
      case FIVE_BALL_MOVING:
        return new FiveBallMovingAutoCommand(swerve, superstructure);
      case SIX_BALL:
        return new SixBallAutoCommand(swerve, superstructure);
      case SIX_BALL_MOVING:
        return new SixBallMovingAutoCommand(swerve, superstructure);
      default:
        return new FiveBallAutoCommand(swerve, superstructure);
    }
  }
}
