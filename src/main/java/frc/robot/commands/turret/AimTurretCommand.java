package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
  private final Supplier<Boolean> ejectSupplier;

  /** Constructor of an AimTurretCommand */
  public AimTurretCommand(
      Turret turret,
      Supplier<Pose2d> estimatedPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Supplier<Rotation2d> headingSupplier,
      Supplier<Boolean> ejectSupplier) {
    // Set turret
    this.turret = turret;

    // Set suppliers
    this.estimatedPoseSupplier = estimatedPoseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.headingSupplier = headingSupplier;
    this.ejectSupplier = ejectSupplier;

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
            .transformBy(new Transform2d(scaledVelocityVector, headingSupplier.get()));

    // Get the angle between the future position of the robot and the goal
    Rotation2d targetAngle = new Transform2d(futurePose, target).getRotation();

    // Rotate that by the angle between the robot front and turret front
    Rotation2d turretAngle = targetAngle.rotateBy(Constants.ROBOT_TO_TURRET);

    // Aim the turret at that angle
    turret.aim(turretAngle.getRadians());
  }
}
