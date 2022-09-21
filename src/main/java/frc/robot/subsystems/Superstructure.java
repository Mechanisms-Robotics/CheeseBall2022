package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/**
 * This class acts as a Superstructure for the intake, processor, and feeder so that we can both
 * intake and shoot at the same time
 */
public class Superstructure extends SubsystemBase {
  // Subsystems
  private final Intake intake;
  private final Processor processor;
  private final Feeder feeder;

  // Suppliers
  private final Supplier<Boolean> turretAtDesiredAngleSupplier;
  private final Supplier<Boolean> hoodAtDesiredAngleSupplier;
  private final Supplier<Boolean> shooterAtDesiredSpeedSupplier;
  private final Supplier<Translation2d> accelerationSupplier;
  private final Supplier<Pose2d> estimatedPoseSupplier;

  // Goal position
  private static final Pose2d GOAL_POSE =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());

  // Flags
  private boolean intaking = false;
  private boolean shooting = false;
  private boolean smart = false;

  /** Constructor for the Superstructure class */
  public Superstructure(
      Intake intake,
      Processor processor,
      Feeder feeder,
      Supplier<Boolean> turretAtDesiredAngleSupplier,
      Supplier<Boolean> hoodAtDesiredAngleSupplier,
      Supplier<Boolean> shooterAtDesiredSpeedSupplier,
      Supplier<Translation2d> accelerationSupplier,
      Supplier<Pose2d> estimatedPoseSupplier) {
    // Set the subsystems
    this.intake = intake;
    this.processor = processor;
    this.feeder = feeder;

    // Set suppliers
    this.turretAtDesiredAngleSupplier = turretAtDesiredAngleSupplier;
    this.hoodAtDesiredAngleSupplier = hoodAtDesiredAngleSupplier;
    this.shooterAtDesiredSpeedSupplier = shooterAtDesiredSpeedSupplier;
    this.accelerationSupplier = accelerationSupplier;
    this.estimatedPoseSupplier = estimatedPoseSupplier;
  }

  /** Deploys and runs the intake, then sets the intaking flag to true */
  public void intake() {
    // Check if the intake is retracted
    if (intake.isRetracted()) {
      // If it is deploy it
      intake.deploy();
    }

    // Run the intake
    intake.intake();

    // Set the intaking flag to true
    this.intaking = true;
  }

  /** Stops and retracts the intake, then sets the intaking flag to false */
  public void stopIntaking() {
    // Stop the intake
    intake.stop();

    // Check if the intake is deployed
    if (intake.isDeployed()) {
      // If it is retract it
      intake.retract();
    }

    // Set the intaking flag to false
    this.intaking = false;
  }

  /** Sets the shooting flag to true */
  public void shoot() {
    // Set the shooting flag to true
    this.shooting = true;

    // Set the smart flag to false
    this.smart = false;
  }

  /** Sets the shooting flag to false */
  public void stopShooting() {
    // Set the shooting flag to false
    this.shooting = false;

    // Set the smart flag to false
    this.smart = false;
  }

  /** Sets the shooting flag and smart flag to true */
  public void smartShoot() {
    // Set the shooting flag to true
    this.shooting = true;

    // Set the smart flag to true
    this.smart = true;
  }

  /** Sets the shooting flag and smart flag to false */
  public void stopSmartShooting() {
    // Set the shooting flag to false
    this.shooting = false;

    // Set the smart flag to false
    this.smart = false;
  }

  /**
   * Runs periodically and contains the logic for how to handle intaking, shooting, and both
   * simultaneously
   */
  public void periodic() {
    // Check what flags are set
    if (this.intaking && !this.shooting) {
      // If only the intaking flag is set, run the processor and feeder in intake mode
      processor.intake();
      feeder.intake();
    } else if (this.shooting) {
      if (this.smart && hasGoodShot()) {
        // If smart shooting is enabled and the robot has a good shot, run the processor and feeder
        // in shooting mode
        processor.shoot();
        feeder.shoot();
      } else if (!this.smart) {
        // If shooting is enabled but the smart flag is not set, run the processor and feeder in
        // shooting mode
        processor.shoot();
        feeder.shoot();
      } else if (this.intaking) {
        // If smart shooting is enabled and the robot does not have a good shot and intaking is
        // enabled, run the processor and feeder in intake mode
        processor.intake();
        feeder.intake();
      } else {
        // If smart shooting is enabled and the robot does not have a good shot and intaking is
        // disabled, stop the processor and feeder
        processor.stop();
        feeder.stop();
      }
    } else {
      // If neither are set stop the processor and feeder
      processor.stop();
      feeder.stop();
    }
  }

  /** Checks if the robot has a good shot */
  private boolean hasGoodShot() {
    // Check if the turret is at it's desired angle
    if (!turretAtDesiredAngleSupplier.get()) {
      // If it isn't return false
      return false;
    }

    // Check if the hood is at it's desired angle
    if (!hoodAtDesiredAngleSupplier.get()) {
      // If it isn't return false
      return false;
    }

    // Check if the shooter is at it's desired RPM
    if (!shooterAtDesiredSpeedSupplier.get()) {
      // If it isn't return false
      return false;
    }

    // Max acceleration and range for a good shot
    double maxAccel = 0.1; // m/s^2
    double maxRange = 5.0; // m

    // Get the acceleration of the robot
    Translation2d acceleration = accelerationSupplier.get();

    // Check if the acceleration exceeds the maximum acceleration
    if (acceleration.getNorm() > maxAccel) {
      // If it does return false
      return false;
    }

    // Get the estimated pose of the robot and calculate a range to the goal
    Pose2d estimatedPose = estimatedPoseSupplier.get();
    double range = new Transform2d(estimatedPose, GOAL_POSE).getTranslation().getNorm();

    // Check if the range exceeds the maximum range
    if (range > maxRange) {
      // If it does return false
      return false;
    }

    // If all tests have passed return true
    return true;
  }

  /** Returns whether the superstructure is intaking or not */
  public boolean isIntaking() {
    // Return the intaking flag
    return this.intaking;
  }

  /** Returns whether the superstructure is shooting or not */
  public boolean isShooting() {
    // Return the shooting flag
    return this.shooting;
  }

  /** Returns whether the superstructure is smart shooting or not */
  public boolean isSmartShooting() {
    // Check if both the shooting and smart flag are true, if so return true
    return (this.shooting && this.smart);
  }
}
