package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;

  /** Runs once on bootup */
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    robotContainer.goalTracker.turnOffLEDs();
  }

  /** Runs periodically after bootup */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** Runs once on disabled */
  @Override
  public void disabledInit() {
    // Turn off the Limelight LEDs
    this.robotContainer.goalTracker.turnOffLEDs();
  }

  /** Runs periodically while disabled */
  @Override
  public void disabledPeriodic() {}

  /** Runs once at the start of autonomous */
  @Override
  public void autonomousInit() {
    // Turn on the Limelight LEDs
    this.robotContainer.goalTracker.turnOnLEDs();

    // Zero the turret
    this.robotContainer.turret.zero();

    // Zero the hood
    this.robotContainer.hood.zero();

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /** Runs periodically during autonomous */
  @Override
  public void autonomousPeriodic() {}

  /** Runs once at the start of teleop */
  @Override
  public void teleopInit() {
    // Turn on the Limelight LEDs
    this.robotContainer.goalTracker.turnOnLEDs();

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** Runs periodically during teleop */
  @Override
  public void teleopPeriodic() {}

  /** Runs once at the start of test mode */
  @Override
  public void testInit() {
    // Turn on the Limelight LEDs
    this.robotContainer.goalTracker.turnOnLEDs();

    // Zero the turret
    this.robotContainer.turret.zero();

    // Zero the hood
    this.robotContainer.hood.zero();

    // Cancel all commands
    CommandScheduler.getInstance().cancelAll();
  }

  /** Runs periodically while test mode is enabled */
  @Override
  public void testPeriodic() {}
}
