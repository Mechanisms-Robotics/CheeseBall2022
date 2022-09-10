package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class StartEndScheduleCommand extends CommandBase {
	private final Command startCommand;
	private final Command endCommand;

	public StartEndScheduleCommand(Command start, Command end) {
		this.startCommand = start;
		this.endCommand = end;
	}

	@Override
	public void initialize() {
		startCommand.initialize();
	}

	@Override
	public void end(boolean interrupted) {
		endCommand.end(false);
	}
}
