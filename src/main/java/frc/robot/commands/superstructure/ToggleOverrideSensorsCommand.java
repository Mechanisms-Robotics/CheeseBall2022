package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

/** This command tells the superstructure to toggle the sensor override */
public class ToggleOverrideSensorsCommand extends InstantCommand {
  /** Constructor of a ToggleOverrideSensorsCommand */
  public ToggleOverrideSensorsCommand(Superstructure superstructure) {
    super(superstructure::toggleOverrideSensors, superstructure);
  }
}
