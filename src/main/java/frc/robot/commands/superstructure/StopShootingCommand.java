package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

/** This command tells the superstructure to stop shooting */
public class StopShootingCommand extends InstantCommand {
  /** Constructor of a StopShootingCommand */
  public StopShootingCommand(Superstructure superstructure) {
    super(superstructure::stopShooting, superstructure);
  }
}
