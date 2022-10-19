package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

/** Varies the shooter RPMs depending on range to the goal */
public class AimShooterWithLLCommand extends CommandBase {
  // Instance of Shooter
  public final Shooter shooter;

  // Suppliers
  private final Supplier<Double> goalTrackerRangeSupplier;
  private final Supplier<Boolean> hasTargetSupplier;

  /** Constructor of an AimShooterWithLLCommand */
  public AimShooterWithLLCommand(
      Shooter shooter,
      Supplier<Double> goalTrackerRangeSupplier,
      Supplier<Boolean> hasTargetSupplier) {
    // Set shooter
    this.shooter = shooter;

    // Set suppliers
    this.goalTrackerRangeSupplier = goalTrackerRangeSupplier;
    this.hasTargetSupplier = hasTargetSupplier;

    // Add the shooter as a requirement
    addRequirements(shooter);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // If the robot does not see a target return
    if (!hasTargetSupplier.get()) {
      return;
    }

    // Vary the shooter RPM based on the calculated range
    shooter.shoot(goalTrackerRangeSupplier.get());
  }
}
