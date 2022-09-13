package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RetractIntakeCommand;
import frc.robot.commands.processor.IntakeProcessorCommand;
import frc.robot.commands.processor.StopProcessorCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;

/** This command retracts the intake, stops the intake, and stops the processor. */
public class StopIntakingCommand extends ParallelCommandGroup {
  /** Constructor of a StopIntakingCommand */
  public StopIntakingCommand(Intake intake, Processor processor) {
    // Add commands to the ParallelCommandGroup
    addCommands(new RetractIntakeCommand(intake), new StopProcessorCommand(processor));
  }
}
