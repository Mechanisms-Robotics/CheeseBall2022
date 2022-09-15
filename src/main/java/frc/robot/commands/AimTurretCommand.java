package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

/** This command aims the turret at the goal */
public class AimTurretCommand extends CommandBase {
  // Instance of Turret
  private final Turret turret;

  // Suppliers
  private final Supplier<Pose2d> estimatedPoseSupplier;

  // Transforms
  private static final Pose2d GOAL_POSE =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());
  private static final Rotation2d ROBOT_TO_TURRET = Rotation2d.fromDegrees(90.0);

  /** Constructor of an AimTurretCommand */
  public AimTurretCommand(Turret turret, Supplier<Pose2d> estimatedPoseSupplier) {
    // Set turret
    this.turret = turret;

    // Set suppliers
    this.estimatedPoseSupplier = estimatedPoseSupplier;

    // Add the turret as a requirement
    addRequirements(turret);
  }

  /** Runs periodically while the command is running */
  @Override
  public void execute() {
    // Gets the estimated pose of the robot from the SwerveDrivePoseEstimator
    Pose2d estimatedPose = estimatedPoseSupplier.get();

    // Get the angle between the robot front and the goal
    Rotation2d targetAngle = new Transform2d(estimatedPose, GOAL_POSE).getRotation();

    // Rotate that by the angle between the robot front and turret front
    Rotation2d turretAngle = targetAngle.rotateBy(ROBOT_TO_TURRET);

    // Aim the turret at that angle
    turret.aim(turretAngle.getRadians());
  }
}
