package frc.robot.commands.processor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Processor;

public class ProcessorRunCommand extends CommandBase {

	private Processor processor;

	public ProcessorRunCommand(Processor processor) {
		this.processor = processor;
		addRequirements(processor);
	}

	@Override
	public void initialize() {
		processor.on();
	}

	@Override
	public void end(boolean interrupted) {
		processor.off();
	}
}
