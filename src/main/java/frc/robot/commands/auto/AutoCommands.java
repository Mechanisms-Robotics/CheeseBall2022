package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public final class AutoCommands {

  public static class ResetPose extends InstantCommand {
    public ResetPose(PathPlannerTrajectory trajectory, Swerve swerve) {
      super(
          () ->
              swerve.setPose(
                  trajectory.getInitialPose(), trajectory.getInitialState().holonomicRotation));
    }
  }

  public static class FollowPathCommand extends SequentialCommandGroup {
    public FollowPathCommand(PathPlannerTrajectory trajectory, Swerve swerve) {
      addCommands(
          new FunctionalCommand(
              () -> swerve.followTrajectory(trajectory),
              () -> {},
              interrupted -> {},
              swerve::isTrajectoryFinished,
              swerve),
          new InstantCommand(swerve::stop, swerve));
    }
  }
}
