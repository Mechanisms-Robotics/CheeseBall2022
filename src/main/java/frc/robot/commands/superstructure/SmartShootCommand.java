package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

/** This command tells the superstructure to smart shoot */
public class SmartShootCommand extends InstantCommand {
  /** Constructor of a SmartShootCommand */
  public SmartShootCommand(Superstructure superstructure) {
    super(superstructure::smartShoot, superstructure);
  }
}
