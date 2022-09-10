package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Feeder;

public class FeederRunCommand extends CommandBase {

	private final Feeder feeder;

	public FeederRunCommand(Feeder feeder) {
		this.feeder = feeder;
		addRequirements(feeder);
	}

	@Override
	public void initialize() {
		feeder.on();
	}

	@Override
	public void end(boolean interrupted) {
		feeder.off();
	}
}
