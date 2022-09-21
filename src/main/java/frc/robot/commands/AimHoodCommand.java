package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import java.util.function.Supplier;

/** This command aims the hood at the goal accounting for the current robot velocity */
public class AimHoodCommand extends CommandBase {
  // Instance of Hood
  private final Hood hood;

  // Suppliers
  private final Supplier<Pose2d> estimatedPoseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private final Supplier<Rotation2d> headingSupplier;

  // Goal position
  private static final Pose2d GOAL_POSE =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());

  // Air Time LOBF
  private static final double AIR_TIME_SLOPE = 0.07;
  private static final double AIR_TIME_INTERCEPT = 0.77;

  /** Constructor of a AimHoodCommand */
  public AimHoodCommand(
      Hood hood,
      Supplier<Pose2d> estimatedPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<Rotation2d> headingSupplier) {
    // Set hood
    this.hood = hood;

    // Set suppliers
    this.estimatedPoseSupplier = estimatedPoseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.headingSupplier = headingSupplier;

    // Add the hood as a requirement
    addRequirements(hood);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Get the chassis speeds of the swerve
    ChassisSpeeds speeds = chassisSpeedsSupplier.get();

    // Convert the chassis speeds to a robot relative velocity vector
    Translation2d velocityVector =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    // Rotate the velocity vector so it is field relative
    velocityVector.rotateBy(headingSupplier.get().unaryMinus());

    // Get the estimated pose of the robot from the SwerveDrivePoseEstimator
    Pose2d estimatedPose = estimatedPoseSupplier.get();

    // Calculate the current range to the target
    double range = new Transform2d(estimatedPose, GOAL_POSE).getTranslation().getNorm();

    // Calculate the air time of the ball from this range and scale the velocity vector
    Translation2d scaledVelocityVector =
        velocityVector.times(AIR_TIME_SLOPE * range + AIR_TIME_INTERCEPT);

    // Calculate the pose of the robot by the time the shot will land
    Pose2d futurePose =
        estimatedPoseSupplier
            .get()
            .transformBy(new Transform2d(scaledVelocityVector, headingSupplier.get()));

    // Calculate what the range to the goal is by the time the shot will land
    double futureRange = GOAL_POSE.minus(futurePose).getTranslation().getNorm();

    // Aim the hood at a calculated angle based on the future range to the goal
    hood.aim(futureRange);
  }
}
