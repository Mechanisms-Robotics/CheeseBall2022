package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import java.util.function.Supplier;

/** This command aims the hood at the goal */
public class AimHoodCommand extends CommandBase {
  // Instance of Hood
  private final Hood hood;

  // Suppliers
  private final Supplier<Pose2d> estimatedPoseSupplier;

  // Transforms
  private static final Pose2d GOAL_POSE =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());

  /** Constructor of a AimHoodCommand */
  public AimHoodCommand(Hood hood, Supplier<Pose2d> estimatedPoseSupplier) {
    // Set hood
    this.hood = hood;

    // Set the estimated pose supplier
    this.estimatedPoseSupplier = estimatedPoseSupplier;

    // Add the hood as a requirement
    addRequirements(hood);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Get the estimated pose of the robot
    Pose2d estimatedPose = estimatedPoseSupplier.get();

    // Calculate the range to the goal
    double range = GOAL_POSE.minus(estimatedPose).getTranslation().getNorm();

    // Aim the hood at a calculated angle based on the range to the goal
    hood.aim(range);
  }
}
