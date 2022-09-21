package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

/**
 * Varies the shooter RPMs depending on range to the goal accounting for the current robot velocity
 */
public class AimShooterCommand extends CommandBase {
  // Instance of Shooter
  public final Shooter shooter;

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

  /** Constructor of an AimShooterCommand */
  public AimShooterCommand(
      Shooter shooter,
      Supplier<Pose2d> estimatedPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<Rotation2d> headingSupplier) {
    // Set shooter
    this.shooter = shooter;

    // Set suppliers
    this.estimatedPoseSupplier = estimatedPoseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.headingSupplier = headingSupplier;

    // Add the shooter as a requirement
    addRequirements(shooter);
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

    // Vary the shooter RPM based on the calculated future range
    shooter.shoot(futureRange);
  }
}
