package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public final class Constants {
    public static final double loopTime = 0.02; // s

    public static final Transform2d fieldRobot =
            new Transform2d(
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)),
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)));
}
