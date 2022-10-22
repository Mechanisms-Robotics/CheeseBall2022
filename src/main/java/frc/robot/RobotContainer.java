package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.colorSensor.ToggleOverrideColorSensorCommand;
import frc.robot.commands.hood.AimHoodCommand;
import frc.robot.commands.hood.AimHoodWithLLCommand;
import frc.robot.commands.shooter.AimShooterCommand;
import frc.robot.commands.shooter.AimShooterWithLLCommand;
import frc.robot.commands.superstructure.EjectCommand;
import frc.robot.commands.superstructure.StopEjectingCommand;
import frc.robot.commands.superstructure.StopUnjammingCommand;
import frc.robot.commands.superstructure.ToggleOverrideSensorsCommand;
import frc.robot.commands.superstructure.UnjamCommand;
import frc.robot.commands.turret.AimTurretCommand;
import frc.robot.commands.superstructure.IntakeCommand;
import frc.robot.commands.goalTracker.PoseEstimateCommand;
import frc.robot.commands.superstructure.ShootCommand;
import frc.robot.commands.superstructure.SmartShootCommand;
import frc.robot.commands.superstructure.StopIntakingCommand;
import frc.robot.commands.superstructure.StopShootingCommand;
import frc.robot.commands.superstructure.StopSmartShootingCommand;
import frc.robot.commands.auto.FiveBallAutoCommand;
import frc.robot.commands.auto.FiveBallMovingAutoCommand;
import frc.robot.commands.auto.SixBallAutoCommand;
import frc.robot.commands.auto.SixBallMovingAutoCommand;
import frc.robot.commands.swerve.DriveTeleopCommand;
import frc.robot.commands.turret.AimTurretWithLLCommand;
import frc.robot.subsystems.ColorSensor;
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
import frc.robot.util.ControllerWrapper.Direction;

/** This class contains all the subsystems, command bindings, and button bindings */
public class RobotContainer {
  // Subsystems
  public final Swerve swerve = new Swerve(PoseEstimateCommand::doesEstimatorHaveVision);
  private final Intake intake = new Intake();
  public final Processor processor = new Processor();
  public final Feeder feeder =
      new Feeder(
          processor.processorSensor::get,
          processor.feederBottomSensor::get,
          processor.feederTopSensor::get);
  public final Turret turret = new Turret();
  public final Shooter shooter = new Shooter();
  public final Hood hood = new Hood();

  // Goal Tracker
  public final GoalTracker goalTracker = new GoalTracker();

  // Superstructure
  private final Superstructure superstructure =
      new Superstructure(
          intake,
          processor,
          feeder,
          turret::atDesiredAngle,
          hood::atDesiredAngle,
          shooter::atDesiredSpeed,
          swerve::getAcceleration,
          swerve::getPose);

  // Color Sensor
  public final ColorSensor colorSensor = new ColorSensor();

  // Controller
  private final ControllerWrapper driverController = new ControllerWrapper(0);

  // Buttons
  private final Button gyroResetButton = new Button(driverController::getShareButton);
  private final Button toggleIntakeButton = new Button(driverController::getLeftTriggerButton);
  private final Button shootButton = new Button(driverController::getRightTriggerButton);
  private final Button toggleSmartShootButton = new Button(driverController::getRightBumperButton);
  private final Button manualEjectButton = new Button(driverController::getXButton);
  private final Button unjamButton = new Button(driverController::getTriangleButton);
  private final Button toggleOverrideProximitySensors =
      new Button(() -> driverController.getPOV() == Direction.Down);
  private final Button toggleOverrideColorSensor =
      new Button(() -> driverController.getPOV() == Direction.Up);

  private final Button toggleAimingMode = new Button(driverController::getRightBumperButton);

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

    // Turn off the Limelight LEDs
    this.goalTracker.turnOffLEDs();
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

    // When the manual eject button is pressed run an EjectCommand
    manualEjectButton.whenPressed(new EjectCommand(superstructure));

    // When the manual eject button is released run a StopEjectingCommand
    manualEjectButton.whenReleased(new StopEjectingCommand(superstructure));

    // When the unjam button is pressed run an UnjamCommand
    unjamButton.whenPressed(new UnjamCommand(superstructure));

    // When the unjam button is released run a StopUnjammingCommand
    unjamButton.whenReleased(new StopUnjammingCommand(superstructure));

    // When the toggle override proximity sensors button is pressed run a
    // ToggleOverrideSensorsCommand
    toggleOverrideProximitySensors.whenPressed(new ToggleOverrideSensorsCommand(superstructure));

    // When the toggle override color sensor button is pressed run a
    // ToggleOverrideColorSensorCommand
    toggleOverrideColorSensor.whenPressed(new ToggleOverrideColorSensorCommand(colorSensor));
  }

  /** Configures the default commands for each subsystem */
  private void configureDefaultCommands() {
    // Set the default swerve command to a DriveTeleopCommand
    swerve.setDefaultCommand(
        new DriveTeleopCommand(
            () -> -driverController.getLeftJoystickX(),
            driverController::getLeftJoystickY,
            () -> -driverController.getRightJoystickX(),
            swerve));

    // Set the default goal tracker command to a PoseEstimateCommand
    goalTracker.setDefaultCommand(
        new PoseEstimateCommand(
            goalTracker, swerve.poseEstimator, turret::getAngle, swerve::getHeading));

    // Set the default turret command to an AimTurretCommand
    turret.setDefaultCommand(
        new AimTurretCommand(
            turret,
            swerve::getPose,
            swerve::getSpeeds,
            swerve::getHeading,
            () -> (manualEjectButton.get() || colorSensor.isWrongColor()),
            swerve::hasBeenLocalized,
            goalTracker::hasSeenTarget,
            () -> goalTracker.getCurrentTarget().angle,
            goalTracker::hasTarget,
            toggleAimingMode::get));

    // Set the default shooter command to an AimShooterCommand
    shooter.setDefaultCommand(
        new AimShooterCommand(
            shooter,
            swerve::getPose,
            swerve::getSpeeds,
            swerve::getHeading,
            () -> (manualEjectButton.get() || colorSensor.isWrongColor()),
            swerve::hasBeenLocalized,
            goalTracker::hasSeenTarget,
            () -> goalTracker.getCurrentTarget().range,
            goalTracker::hasTarget,
            toggleAimingMode::get));

    // t Set the default hood command to an AimHoodCommand
    hood.setDefaultCommand(
        new AimHoodCommand(
            hood,
            swerve::getPose,
            swerve::getSpeeds,
            swerve::getHeading,
            () -> (manualEjectButton.get() || colorSensor.isWrongColor()),
            swerve::hasBeenLocalized,
            goalTracker::hasSeenTarget,
            () -> goalTracker.getCurrentTarget().range,
            goalTracker::hasTarget,
            toggleAimingMode::get));

    //    shooter.setDefaultCommand(
    //        new AimShooterWithLLCommand(
    //            shooter, () -> goalTracker.getCurrentTarget().range, goalTracker::hasTarget));
    //    hood.setDefaultCommand(
    //        new AimHoodWithLLCommand(
    //            hood, () -> goalTracker.getCurrentTarget().range, goalTracker::hasTarget));
    //    turret.setDefaultCommand(
    //        new AimTurretWithLLCommand(
    //            turret, () -> goalTracker.getCurrentTarget().angle, goalTracker::hasTarget));
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
