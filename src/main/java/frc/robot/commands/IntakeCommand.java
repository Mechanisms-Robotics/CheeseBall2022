package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

/** This command tells the superstructure to intake */
public class IntakeCommand extends InstantCommand {
  /** Constructor of an IntakeCommand */
  public IntakeCommand(Superstructure superstructure) {
    super(superstructure::intake, superstructure);
  }
}
