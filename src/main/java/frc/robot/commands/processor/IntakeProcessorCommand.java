package frc.robot.commands.processor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Processor;

/** This command runs the processor in order to intake a ball */
public class IntakeProcessorCommand extends CommandBase {
  // Instance of Processor subsystem
  private final Processor processor;

  /** Constructor of an IntakeProcessorCommand */
  public IntakeProcessorCommand(Processor processor) {
    // Set processor
    this.processor = processor;

    // Add the processor as a requirement
    addRequirements(processor);
  }

  /** Runs when the command is first initialized */
  @Override
  public void initialize() {
    // Run the processor
    processor.intake();
  }
}
