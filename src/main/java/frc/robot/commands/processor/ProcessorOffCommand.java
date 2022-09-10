package frc.robot.commands.processor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Processor;

public class ProcessorOffCommand extends CommandBase {

	private Processor processor;

	public ProcessorOffCommand(Processor processor) {
		this.processor = processor;
		addRequirements(processor);
	}

	@Override
	public void initialize() {
		processor.off();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
