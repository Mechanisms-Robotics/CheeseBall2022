package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class GoalTracker extends SubsystemBase {
  // Instance of limelight
  private final PhotonCamera limelight;

  // Limelight constants
  private static final double TARGET_HEIGHT = 2.64; // meters
  private static final double CAMERA_HEIGHT = 1.0; // meters
  private static final double CAMERA_PITCH = Math.toRadians(60.0); // radians

  /** This class is used as a structure for target data */
  public static class TargetData {
    public boolean hasTarget = false;
    public Rotation2d angle;
    public double range;
  }

  // Current Limelight target
  private final TargetData currentTarget = new TargetData();

  /** Constructor for the GoalTracker class */
  public GoalTracker() {
    // Instantiate the limelight and limelight network table
    this.limelight = new PhotonCamera("limelight");
  }

  /**
   * Runs periodically and contains the logic for getting the best target from the limelight and
   * calculating the range to it
   */
  @Override
  public void periodic() {
    // Get the latest result from the Limelight
    PhotonPipelineResult limelightResult = this.limelight.getLatestResult();

    // Check if there are any targets in the result
    if (limelightResult.hasTargets()) {
      // If there are get the best target
      PhotonTrackedTarget bestTarget = limelightResult.getBestTarget();

      // Set the current target data to that of the best target
      this.currentTarget.hasTarget = true;
      this.currentTarget.angle = Rotation2d.fromDegrees(-bestTarget.getYaw());
      this.currentTarget.range =
          photonRangeToRealRange(
              PhotonUtils.calculateDistanceToTargetMeters(
                  CAMERA_HEIGHT,
                  TARGET_HEIGHT,
                  CAMERA_PITCH,
                  Math.toRadians(bestTarget.getPitch())));

      // Output the current target data to SmartDashboard
      SmartDashboard.putBoolean("Has Target", this.currentTarget.hasTarget);
      SmartDashboard.putNumber("Target Angle", this.currentTarget.angle.getDegrees());
      SmartDashboard.putNumber("Target Range", this.currentTarget.range);
    } else {
      // If there are no targets set the current target has target flag to false
      this.currentTarget.hasTarget = false;

      // Output the current target has target value to SmartDashboard
      SmartDashboard.putBoolean("Has Target", this.currentTarget.hasTarget);
    }
  }

  /** Converts the range PhotonVision calculates to a real range to the goal */
  private double photonRangeToRealRange(double photonRange) {
    double realRange = photonRange;
    return photonRange;
  }

  /** Turns on the Limelight's LEDs */
  public void turnOnLEDs() {
    // Set the limelight LED mode to on
    this.limelight.setLED(VisionLEDMode.kOn);
  }

  /** Turns off the Limelight's LEDs */
  public void turnOffLEDs() {
    // Set the limelight LED mode to off
    this.limelight.setLED(VisionLEDMode.kOff);
  }

  /** Returns the current target */
  public TargetData getCurrentTarget() {
    // Return currentTarget
    return this.currentTarget;
  }
}
