package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import java.util.function.Supplier;

/** Varies the hood angle depending on range to the goal */
public class AimHoodWithLLCommand extends CommandBase {
  // Instance of Shooter
  public final Hood hood;

  // Suppliers
  private final Supplier<Double> goalTrackerRangeSupplier;
  private final Supplier<Boolean> hasTargetSupplier;

  /** Constructor of an AimHoodWithLLCommand */
  public AimHoodWithLLCommand(
      Hood hood, Supplier<Double> goalTrackerRangeSupplier, Supplier<Boolean> hasTargetSupplier) {
    // Set hood
    this.hood = hood;

    // Set suppliers
    this.goalTrackerRangeSupplier = goalTrackerRangeSupplier;
    this.hasTargetSupplier = hasTargetSupplier;

    // Add the hood as a requirement
    addRequirements(hood);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // If the robot does not see a target return
    if (!hasTargetSupplier.get()) {
      return;
    }

    // Vary the hood angle based on the calculated range
    hood.aim(goalTrackerRangeSupplier.get());
  }
}
