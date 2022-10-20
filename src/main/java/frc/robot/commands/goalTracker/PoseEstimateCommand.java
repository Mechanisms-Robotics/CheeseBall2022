package frc.robot.commands.goalTracker;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GoalTracker;
import frc.robot.subsystems.GoalTracker.TargetData;
import java.util.function.Supplier;
import javax.xml.crypto.dsig.Transform;
import org.photonvision.PhotonUtils;

/**
 * This command uses vision localization to estimate a robot pose and updates the
 * SwerveDrivePoseEstimator with the estimated vision pose
 */
public class PoseEstimateCommand extends CommandBase {
  private static final double TARGET_HEIGHT = 2.64; // meters
  private static final double CAMERA_HEIGHT = 0.99; // meters
  private static final double CAMERA_PITCH = Math.toRadians(20.0); // radians

  // Instance of GoalTracker
  private final GoalTracker goalTracker;

  // Instance of SwerveDrivePoseEstimator
  private final SwerveDrivePoseEstimator poseEstimator;

  // Suppliers
  private final Supplier<Double> turretAngleSupplier;
  private final Supplier<Rotation2d> gyroAngleSupplier;

  // Transforms
  private static final Rotation2d TURRET_TO_ROBOT = Rotation2d.fromDegrees(90.0);
  private static final Pose2d GOAL_POSE =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());

  private static final Field2d visionPoseField2d = new Field2d();

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

      visionPoseField2d.setRobotPose(estimatedPose);
    }

    // Put the Field2d instance on the SmartDashboard
    SmartDashboard.putData("Vision Field", visionPoseField2d);
  }

  Transform2d cameraToTurret = new Transform2d(new Translation2d(0.18415, 0.0), new Rotation2d());
  Transform2d turretToRobot = new Transform2d(new Translation2d(0.2032, 0.0), new Rotation2d());

  Transform2d cameraToRobot = turretToRobot.plus(turretToRobot);

  /** Estimates the robot pose based on the angle and range from the Limelight to the target */
  private Pose2d estimateRobotPose(Rotation2d angle, double range) {
    return PhotonUtils.estimateFieldToRobot(
        CAMERA_HEIGHT,
        TARGET_HEIGHT,
        CAMERA_PITCH,
        goalTracker.getCurrentTarget().pitch,
        goalTracker.getCurrentTarget().angle,
        gyroAngleSupplier
            .get()
            .rotateBy(
                Rotation2d.fromDegrees(-90.0).rotateBy(new Rotation2d(turretAngleSupplier.get()))),
        GOAL_POSE,
        turretToRobot);
  }
}
