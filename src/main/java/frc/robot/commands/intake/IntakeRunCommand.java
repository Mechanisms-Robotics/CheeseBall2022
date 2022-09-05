package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRunCommand extends CommandBase {

	private Intake intake;

	public IntakeRunCommand(Intake intake) {
		this.intake = intake;
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.intake();
	}

	@Override
	public void end(boolean interrupted) {
		intake.stop();
	}
}
