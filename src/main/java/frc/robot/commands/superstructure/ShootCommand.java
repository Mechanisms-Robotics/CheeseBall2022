package frc.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

/** This command tells the superstructure to shoot */
public class ShootCommand extends InstantCommand {
  /** Constructor of a ShootCommand */
  public ShootCommand(Superstructure superstructure) {
    super(superstructure::shoot, superstructure);
  }
}
