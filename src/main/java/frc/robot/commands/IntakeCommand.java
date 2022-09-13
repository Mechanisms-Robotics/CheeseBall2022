package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.processor.IntakeProcessorCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;

/** This command deploys the intake, runs the intake, and runs the processor. */
public class IntakeCommand extends ParallelCommandGroup {
  /** Constructor of an IntakeCommand */
  public IntakeCommand(Intake intake, Processor processor) {
    // Add commands to the ParallelCommandGroup
    addCommands(new DeployIntakeCommand(intake), new IntakeProcessorCommand(processor));
  }
}
