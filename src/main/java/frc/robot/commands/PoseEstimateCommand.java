package frc.robot.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GoalTracker;
import frc.robot.subsystems.GoalTracker.TargetData;
import java.util.function.Supplier;

/**
 * This command uses vision localization to estimate a robot pose and updates the
 * SwerveDrivePoseEstimator with the estimated vision pose
 */
public class PoseEstimateCommand extends CommandBase {
  // Instance of GoalTracker
  private final GoalTracker goalTracker;

  // Instance of SwerveDrivePoseEstimator
  private final SwerveDrivePoseEstimator poseEstimator;

  // Suppliers
  private final Supplier<Double> turretAngleSupplier;
  private final Supplier<Rotation2d> gyroAngleSupplier;

  // Transforms
  private static final Rotation2d TURRET_TO_ROBOT = Rotation2d.fromDegrees(-90.0);
  private static final Pose2d GOAL_POSE =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());

  /** Constructor of a PoseEstimateCommand */
  public PoseEstimateCommand(
      GoalTracker goalTracker,
      SwerveDrivePoseEstimator poseEstimator,
      Supplier<Double> turretAngleSupplier,
      Supplier<Rotation2d> gyroAngleSupplier) {
    // Set goal tracker
    this.goalTracker = goalTracker;

    // Set pose estimator
    this.poseEstimator = poseEstimator;

    // Set suppliers
    this.turretAngleSupplier = turretAngleSupplier;
    this.gyroAngleSupplier = gyroAngleSupplier;

    // Add the goal tracker as a requirement
    addRequirements(goalTracker);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Get the current target from the goal tracker
    TargetData currentTarget = goalTracker.getCurrentTarget();

    // Check if it has a valid target
    if (currentTarget.hasTarget) {
      // If it does estimate the robot pose based on the angle and range to the target
      Pose2d estimatedPose = estimateRobotPose(currentTarget.angle, currentTarget.range);

      // Add the estimated pose to the pose estimator
      poseEstimator.addVisionMeasurement(estimatedPose, Timer.getFPGATimestamp());
    }
  }

  /** Estimates the robot pose based on the angle and range from the Limelight to the target */
  private Pose2d estimateRobotPose(Rotation2d angle, double range) {
    // Get the current turret angle
    Rotation2d turretAngle = new Rotation2d(turretAngleSupplier.get());

    // Rotate it by the angle to the target
    Rotation2d targetTurretAngle = turretAngle.rotateBy(angle);

    // Rotate that by TURRET_TO_ROBOT to make the angle robot relative
    Rotation2d targetRobotAngle = targetTurretAngle.rotateBy(TURRET_TO_ROBOT);

    // Rotate that by the current gyro angle to make the angle field relative
    Rotation2d targetFieldAngle = targetRobotAngle.rotateBy(gyroAngleSupplier.get());

    // Get the position of the goal relative to the robot as a Transform2d
    Transform2d targetRobotTransform =
        new Transform2d(
            new Translation2d(
                -range * Math.cos(targetFieldAngle.getRadians()),
                -range * Math.sin(targetFieldAngle.getRadians())),
            new Rotation2d());

    // Return the position of the robot based off of the known position of the goal
    return GOAL_POSE.transformBy(targetRobotTransform);
  }
}
