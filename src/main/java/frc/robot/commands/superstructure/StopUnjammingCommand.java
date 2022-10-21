package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

/** This command tells the superstructure to stop unjamminf */
public class StopUnjammingCommand extends InstantCommand {
  /** Constructor of a StopUnjammingCommand */
  public StopUnjammingCommand(Superstructure superstructure) {
    super(superstructure::stopUnjamming, superstructure);
  }
}
