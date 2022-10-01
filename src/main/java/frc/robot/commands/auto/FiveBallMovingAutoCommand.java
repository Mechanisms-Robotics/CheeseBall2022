package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.superstructure.IntakeCommand;
import frc.robot.commands.superstructure.SmartShootCommand;
import frc.robot.commands.auto.AutoCommands.FollowPathCommand;
import frc.robot.commands.auto.AutoCommands.ResetPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

/**
 * This is a 5 ball auto that shoots balls while moving, it starts in the tarmac, picks up the two
 * closest balls, then grabs two from the HP station, and then drives to a "prime" position to make
 * it easy for the driver to grab a ball right when teleop starts
 */
public class FiveBallMovingAutoCommand extends SequentialCommandGroup {
  // Max velocity and acceleration
  private static final double MAX_VEL = 2.0; // m/s
  private static final double MAX_ACCEL = 4.0; // m/s

  // Paths
  private static final PathPlannerTrajectory tarmacToThirdBall =
      PathPlanner.loadPath("TarmacToThirdBall", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory thirdBallToHP =
      PathPlanner.loadPath("ThirdBallToHP", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory hpToShot =
      PathPlanner.loadPath("HPToShot", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory shotToPrime =
      PathPlanner.loadPath("ShotToPrime", MAX_VEL, MAX_ACCEL);

  /** Constructor of a FiveBallMovingAutoCommand */
  public FiveBallMovingAutoCommand(Swerve swerve, Superstructure superstructure) {
    addCommands(
        // Reset the pose, start intaking, and start smart shooting
        new ParallelCommandGroup(
            new ResetPose(tarmacToThirdBall, swerve),
            new IntakeCommand(superstructure),
            new SmartShootCommand(superstructure)),
        // Follow the TarmacToThirdBall path for 2.73 seconds
        new FollowPathCommand(tarmacToThirdBall, swerve),
        // Follow the ThirdBallToHP path for 2.48 seconds
        new FollowPathCommand(thirdBallToHP, swerve),
        // Wait 1 second
        new WaitCommand(1.0),
        // Follow the HPToShot path for 2.48 seconds
        new FollowPathCommand(hpToShot, swerve),
        // Follow the ShotToPrime path for 2.08 seconds
        new FollowPathCommand(shotToPrime, swerve));
  }
}
