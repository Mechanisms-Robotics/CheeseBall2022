package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StopShootingCommand;
import frc.robot.commands.auto.AutoCommands.FollowPathCommand;
import frc.robot.commands.auto.AutoCommands.ResetPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

/**
 * This is a 5 ball auto, that starts in the tarmac, picks up the two closest balls, shoots all
 * three, then grabs two from the HP station, drives back and shoots those two, and then drives to a
 * "prime" position to make it easy for the driver to grab a ball right when teleop starts
 */
public class FiveBallAutoCommand extends SequentialCommandGroup {
  // Max velocity and acceleration
  private static final double MAX_VEL = 4.0; // m/s
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

  /** Constructor of a FiveBallAutoCommand */
  public FiveBallAutoCommand(Swerve swerve, Superstructure superstructure) {
    addCommands(
        new ParallelCommandGroup(
            new ResetPose(tarmacToThirdBall, swerve), new IntakeCommand(superstructure)),
        new FollowPathCommand(tarmacToThirdBall, swerve),
        new ShootCommand(superstructure),
        new WaitCommand(2.0),
        new StopShootingCommand(superstructure),
        new FollowPathCommand(thirdBallToHP, swerve),
        new WaitCommand(1.0),
        new FollowPathCommand(hpToShot, swerve),
        new ShootCommand(superstructure),
        new WaitCommand(2.0),
        new StopShootingCommand(superstructure),
        new FollowPathCommand(shotToPrime, swerve));
  }
}
