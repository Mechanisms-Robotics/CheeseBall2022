package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.superstructure.IntakeCommand;
import frc.robot.commands.superstructure.ShootCommand;
import frc.robot.commands.superstructure.StopShootingCommand;
import frc.robot.commands.auto.AutoCommands.FollowPathCommand;
import frc.robot.commands.auto.AutoCommands.ResetPose;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

/**
 * This is a 6 ball auto, that starts in the tarmac, picks up the two closest balls, shoots all
 * three, then grabs two from the HP station, drives over to the far ball and picks it up, and then
 * shoots all three
 */
public class SixBallAutoCommand extends SequentialCommandGroup {
  // Max velocity and acceleration
  private static final double MAX_VEL = 2.5; // m/s
  private static final double MAX_ACCEL = 4.0; // m/s

  // Paths
  private static final PathPlannerTrajectory tarmacToThirdBall =
      PathPlanner.loadPath("TarmacToThirdBall", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory thirdBallToHP =
      PathPlanner.loadPath("ThirdBallToHP", MAX_VEL, MAX_ACCEL);
  private static final PathPlannerTrajectory hpToSixthBall =
      PathPlanner.loadPath("HPToSixthBall", MAX_VEL, MAX_ACCEL);

  /** Constructor of a SixBallAutoCommand */
  public SixBallAutoCommand(Swerve swerve, Superstructure superstructure) {
    addCommands(
        // Reset the pose and start intaking
        new ParallelCommandGroup(
            new ResetPose(tarmacToThirdBall, swerve), new IntakeCommand(superstructure)),
        // Follow the TarmacToThirdBall path for 3.07 seconds
        new FollowPathCommand(tarmacToThirdBall, swerve),
        // Start shooting
        new ShootCommand(superstructure),
        // Wait 2 seconds
        new WaitCommand(2.0),
        // Stop shooting
        new StopShootingCommand(superstructure),
        // Follow the ThirdBallToHP path for 2.17 seconds
        new FollowPathCommand(thirdBallToHP, swerve),
        // Wait 1 second
        new WaitCommand(2.0),
        // Follow the HPToSixthBall path for 2.87 seconds
        new FollowPathCommand(hpToSixthBall, swerve),
        // Start shooting
        new ShootCommand(superstructure),
        // Wait 2 seconds
        new WaitCommand(2.0),
        // Stop shooting
        new StopShootingCommand(superstructure));
  }
}
