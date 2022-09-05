package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeDeployCommand extends CommandBase {

	private final Intake intake;

	public IntakeDeployCommand(Intake intake) {
		this.intake = intake;
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.deploy();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
