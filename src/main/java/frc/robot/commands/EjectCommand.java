package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Superstructure;

/** This command tells the superstructure to eject */
public class EjectCommand extends InstantCommand {
  /** Constructor of an EjectCommand */
  public EjectCommand(Superstructure superstructure) {
    super(superstructure::eject, superstructure);
  }
}
