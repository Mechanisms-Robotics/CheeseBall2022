package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/** This command deploys the intake and then runs it */
public class DeployIntake extends CommandBase {
  // Instance of Intake subsystem
  private final Intake intake;

  /** Constructor of a DeployIntake command */
  public DeployIntake(Intake intake) {
    // Set intake
    this.intake = intake;

    // Add the intake as a requirement
    addRequirements(intake);
  }

  /** Runs when the command is first initialized */
  @Override
  public void initialize() {
    // If the intake is retracted, deploy it
    if (intake.isRetracted()) {
      intake.deploy();
    }

    // Run the intake
    intake.intake();
  }
}
