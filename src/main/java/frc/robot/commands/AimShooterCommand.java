package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

/** Varies the shooter RPMs depending on range to the goal */
public class AimShooterCommand extends CommandBase {
  // Instance of Shooter
  public final Shooter shooter;

  // Suppliers
  private final Supplier<Pose2d> estimatedPoseSupplier;

  // Transforms
  private static final Pose2d GOAL_POSE =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());

  /** Constructor of an AimShooterCommand */
  public AimShooterCommand(Shooter shooter, Supplier<Pose2d> estimatedPoseSupplier) {
    // Set shooter
    this.shooter = shooter;

    // Set estimated pose supplier
    this.estimatedPoseSupplier = estimatedPoseSupplier;

    // Add the shooter as a requirement
    addRequirements(shooter);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Get the estimated pose of the robot
    Pose2d estimatedPose = estimatedPoseSupplier.get();

    // Calculate the range to the goal
    double range = GOAL_POSE.minus(estimatedPose).getTranslation().getNorm();

    // Vary the shooter RPM based on the calculated range
    shooter.shoot(range);
  }
}
