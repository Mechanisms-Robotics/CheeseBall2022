package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

/** This command tells the superstructure to stop ejecting */
public class StopEjectingCommand extends InstantCommand {
  /** Constructor of a StopEjectingCommand */
  public StopEjectingCommand(Superstructure superstructure) {
    super(superstructure::stopEjecting, superstructure);
  }
}
