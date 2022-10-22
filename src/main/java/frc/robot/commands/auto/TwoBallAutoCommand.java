package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.auto.AutoCommands.FollowPathCommand;
import frc.robot.commands.auto.AutoCommands.ResetPose;
import frc.robot.commands.superstructure.IntakeCommand;
import frc.robot.commands.superstructure.ShootCommand;
import frc.robot.commands.superstructure.StopShootingCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Swerve;

public class TwoBallAutoCommand extends SequentialCommandGroup {
  // Max velocity and acceleration
  private static final double MAX_VEL = 2.5; // m/s
  private static final double MAX_ACCEL = 4.0; // m/s

  // Paths
  private static final PathPlannerTrajectory tarmacTwoBall =
      PathPlanner.loadPath("TarmacTwoBall", MAX_VEL, MAX_ACCEL);

  /** Constructor of a TwoBallAutoCommand */
  public TwoBallAutoCommand(Swerve swerve, Superstructure superstructure) {
    addCommands(
        // Reset the pose and start intaking
        new ParallelCommandGroup(
            new ResetPose(tarmacTwoBall, swerve), new IntakeCommand(superstructure)),
        // Follow the TarmacToThirdBall path for 1.03 seconds
        new FollowPathCommand(tarmacTwoBall, swerve),
        // Start shooting
        new ShootCommand(superstructure),
        // Wait 5 seconds
        new WaitCommand(5.0),
        // Stop shooting
        new StopShootingCommand(superstructure));
  }
}
