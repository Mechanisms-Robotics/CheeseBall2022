package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

/** This command tells the superstructure to unjam */
public class UnjamCommand extends InstantCommand {
  /** Constructor of an UnjamCommand */
  public UnjamCommand(Superstructure superstructure) {
    super(superstructure::unjam, superstructure);
  }
}
