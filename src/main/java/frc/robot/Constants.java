package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

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

  // Robot to turret transform
  public static final Rotation2d ROBOT_TO_TURRET = Rotation2d.fromDegrees(-90.0);

  // Goal pose
  public static final Pose2d GOAL_POSE =
      new Pose2d(new Translation2d(8.23, 4.12), new Rotation2d());

  // Eject pose
  public static final Pose2d EJECT_POSE =
      new Pose2d(new Translation2d(3.50, 6.75), new Rotation2d());

  // Air Time LOBF parameters
  public static final double AIR_TIME_SLOPE = 0.07;
  public static final double AIR_TIME_INTERCEPT = 0.77;
}
