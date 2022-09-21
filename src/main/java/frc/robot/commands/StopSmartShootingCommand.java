package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

/** This command tells the superstructure to stop smart shooting */
public class StopSmartShootingCommand extends InstantCommand {
  /** Constructor of a StopSmartShootingCommand */
  public StopSmartShootingCommand(Superstructure superstructure) {
    super(superstructure::stopSmartShooting, superstructure);
  }
}
