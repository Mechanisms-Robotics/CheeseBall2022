package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRetractCommand extends CommandBase {

	private final Intake intake;
	public IntakeRetractCommand(Intake intake) {
		this.intake = intake;
		addRequirements(intake);
	}

	@Override
	public void initialize() {
		intake.retract();
	}

	@Override
	public boolean isFinished() {
		return true;
	}

}
