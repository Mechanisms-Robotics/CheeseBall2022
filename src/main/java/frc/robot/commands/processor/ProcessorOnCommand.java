package frc.robot.commands.processor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Processor;

public class ProcessorOnCommand extends CommandBase {

	private Processor processor;

	public ProcessorOnCommand(Processor processor) {
		this.processor = processor;
		addRequirements(processor);
	}

	@Override
	public void initialize() {
		processor.on();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
