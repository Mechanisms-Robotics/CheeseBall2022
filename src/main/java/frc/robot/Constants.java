package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/** This class contains all the global constant values * */
public final class Constants {
  // The amount of time to wait for a response before timing out
  public static final int startupCanTimeout = 255; // sec

  // The amount of time between loops
  public static final double loopTime = 0.02; // sec

  // Used to correct any weird odometry rotations
  public static final Transform2d fieldRobot =
      new Transform2d(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)),
          new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)));
}
