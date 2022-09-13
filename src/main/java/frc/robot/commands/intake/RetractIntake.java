package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/** This command retracts the intake and then stops it */
public class RetractIntake extends CommandBase {
  // Instance of Intake subsystem
  private final Intake intake;

  /** Constructor of a RetractIntake command */
  public RetractIntake(Intake intake) {
    // Set intake
    this.intake = intake;

    // Add the intake as a requirement
    addRequirements(intake);
  }

  /** Runs when the command is first initialized */
  @Override
  public void initialize() {
    // If the intake is deployed, retract it
    if (intake.isDeployed()) {
      intake.retract();
    }

    // Stops the intake
    intake.stop();
  }
}
