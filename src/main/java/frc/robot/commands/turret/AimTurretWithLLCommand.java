package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** This command aims the turret at the goal */
public class AimTurretWithLLCommand extends CommandBase {
  // Instance of Turret
  private final Turret turret;

  // Suppliers
  private final Supplier<Rotation2d> goalTrackerAngleSupplier;
  private final Supplier<Boolean> hasTargetSupplier;

  /** Constructor of an AimTurretWithLLCommand */
  public AimTurretWithLLCommand(
      Turret turret,
      Supplier<Rotation2d> goalTrackerAngleSupplier,
      Supplier<Boolean> hasTargetSupplier) {
    // Set turret
    this.turret = turret;

    // Set suppliers
    this.goalTrackerAngleSupplier = goalTrackerAngleSupplier;
    this.hasTargetSupplier = hasTargetSupplier;

    // Add the turret as a requirement
    addRequirements(turret);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // If the robot does not see a target return
    if (!hasTargetSupplier.get()) {
      return;
    }

    // Aim the turret at that angle
    turret.aim(turret.getAngle() + goalTrackerAngleSupplier.get().getRadians());
  }
}
