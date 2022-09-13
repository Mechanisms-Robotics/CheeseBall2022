package frc.robot.commands.processor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Processor;

/** This command stops the processor */
public class StopProcessorCommand extends CommandBase {
  // Instance of Processor subsystem
  private final Processor processor;

  /** Constructor of an StopProcessorCommand */
  public StopProcessorCommand(Processor processor) {
    // Set processor
    this.processor = processor;

    // Add the processor as a requirement
    addRequirements(processor);
  }

  /** Runs when the command is first initialized */
  @Override
  public void initialize() {
    // Stop the processor
    processor.stop();
  }
}
