package frc.robot.commands.hood;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
  private final Supplier<Boolean> ejectSupplier;
  private final Supplier<Boolean> hasBeenLocalizedSupplier;
  private final Supplier<Boolean> hasSeenTargetSupplier;
  private final Supplier<Double> goalTrackerRangeSupplier;
  private final Supplier<Boolean> hasTargetSupplier;
  private final Supplier<Boolean> toggleAimingMode;

  // Odometry mode flag
  private boolean odometryMode = true;

  // Last toggle value
  private boolean lastToggleValue = false;

  /** Constructor of a AimHoodCommand */
  public AimHoodCommand(
      Hood hood,
      Supplier<Pose2d> estimatedPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<Rotation2d> headingSupplier,
      Supplier<Boolean> ejectSupplier,
      Supplier<Boolean> hasBeenLocalizedSupplier,
      Supplier<Boolean> hasSeenTargetSupplier,
      Supplier<Double> goalTrackerRangeSupplier,
      Supplier<Boolean> hasTargetSupplier,
      Supplier<Boolean> toggleAimingMode) {
    // Set hood
    this.hood = hood;

    // Set suppliers
    this.estimatedPoseSupplier = estimatedPoseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.headingSupplier = headingSupplier;
    this.ejectSupplier = ejectSupplier;
    this.hasBeenLocalizedSupplier = hasBeenLocalizedSupplier;
    this.hasSeenTargetSupplier = hasSeenTargetSupplier;
    this.goalTrackerRangeSupplier = goalTrackerRangeSupplier;
    this.hasTargetSupplier = hasTargetSupplier;
    this.toggleAimingMode = toggleAimingMode;

    // Add the hood as a requirement
    addRequirements(hood);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    if (toggleAimingMode.get() && !lastToggleValue) {
      this.odometryMode = !this.odometryMode;
      this.lastToggleValue = true;
    } else if (!toggleAimingMode.get() && lastToggleValue) {
      this.lastToggleValue = false;
    }

    if (this.odometryMode) {
      // If the robot has not been localized and not seen a target return
      if (!hasBeenLocalizedSupplier.get() && !hasSeenTargetSupplier.get()) {
        return;
      }

      // Get the chassis speeds of the swerve
      ChassisSpeeds speeds = chassisSpeedsSupplier.get();

      // Convert the chassis speeds to a robot relative velocity vector
      Translation2d velocityVector =
          new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

      // Rotate the velocity vector so it is field relative
      velocityVector.rotateBy(headingSupplier.get().unaryMinus());

      // Get the estimated pose of the robot from the SwerveDrivePoseEstimator
      Pose2d estimatedPose = estimatedPoseSupplier.get();

      // Initialize target
      Pose2d target;

      // Check if the eject value is true
      if (ejectSupplier.get()) {
        // If it is set target to the eject pose
        target = Constants.EJECT_POSE;
      } else {
        // If it isn't set target to the goal pose
        target = Constants.GOAL_POSE;
      }

      // Calculate the current range to the target
      double range = new Transform2d(estimatedPose, target).getTranslation().getNorm();

      // Calculate the air time of the ball from this range and scale the velocity vector
      Translation2d scaledVelocityVector =
          velocityVector.times(Constants.AIR_TIME_SLOPE * range + Constants.AIR_TIME_INTERCEPT);

      // Calculate the pose of the robot by the time the shot will land
      Pose2d futurePose =
          estimatedPoseSupplier
              .get()
              .transformBy(new Transform2d(scaledVelocityVector, new Rotation2d()));

      // Calculate what the range to the target is by the time the shot will land
      double futureRange = target.minus(futurePose).getTranslation().getNorm();

      // Aim the hood at a calculated angle based on the future range to the goal
      hood.aim(futureRange);
    } else {
      // If the robot does not see a target return
      if (!hasTargetSupplier.get()) {
        return;
      }

      // Vary the hood angle based on the calculated range
      hood.aim(goalTrackerRangeSupplier.get());
    }
  }
}
