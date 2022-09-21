package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** This command aims the turret at the goal accounting for the current robot velocity */
public class AimTurretCommand extends CommandBase {
  // Instance of Turret
  private final Turret turret;

  // Suppliers
  private final Supplier<Pose2d> estimatedPoseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private final Supplier<Rotation2d> headingSupplier;

  // Goal position
  private static final Pose2d GOAL_POSE =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());

  // Transforms
  private static final Rotation2d ROBOT_TO_TURRET = Rotation2d.fromDegrees(-90.0);

  // Air Time LOBF
  private static final double AIR_TIME_SLOPE = 0.07;
  private static final double AIR_TIME_INTERCEPT = 0.77;

  /** Constructor of an AimTurretCommand */
  public AimTurretCommand(
      Turret turret,
      Supplier<Pose2d> estimatedPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<Rotation2d> headingSupplier) {
    // Set turret
    this.turret = turret;

    // Set suppliers
    this.estimatedPoseSupplier = estimatedPoseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.headingSupplier = headingSupplier;

    // Add the turret as a requirement
    addRequirements(turret);
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

    // Get the angle between the future position of the robot and the goal
    Rotation2d targetAngle = new Transform2d(futurePose, GOAL_POSE).getRotation();

    // Rotate that by the angle between the robot front and turret front
    Rotation2d turretAngle = targetAngle.rotateBy(ROBOT_TO_TURRET);

    // Aim the turret at that angle
    turret.aim(turretAngle.getRadians());
  }
}
