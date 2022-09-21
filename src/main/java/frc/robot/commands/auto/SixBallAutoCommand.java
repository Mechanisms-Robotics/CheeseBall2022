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
 * This is a 6 ball auto, that starts in the tarmac, picks up the two closest balls, shoots all
 * three, then grabs two from the HP station, drives over to the far ball and picks it up, and then
 * shoots all three
 */
public class SixBallAutoCommand extends SequentialCommandGroup {
  // Max velocity and acceleration
  private static final double MAX_VEL = 4.0; // m/s
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
        new ParallelCommandGroup(
            new ResetPose(tarmacToThirdBall, swerve), new IntakeCommand(superstructure)),
        new FollowPathCommand(tarmacToThirdBall, swerve),
        new ShootCommand(superstructure),
        new WaitCommand(2.0),
        new StopShootingCommand(superstructure),
        new FollowPathCommand(thirdBallToHP, swerve),
        new WaitCommand(1.0),
        new FollowPathCommand(hpToSixthBall, swerve),
        new ShootCommand(superstructure),
        new WaitCommand(2.0),
        new StopShootingCommand(superstructure));
  }
}
