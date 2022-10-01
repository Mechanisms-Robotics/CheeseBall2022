package frc.robot.commands.colorSensor;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ColorSensor;

/** This command tells the color sensor to toggle the sensor override */
public class ToggleOverrideColorSensorCommand extends InstantCommand {
  /** Constructor of a ToggleOverrideColorSensorCommand */
  public ToggleOverrideColorSensorCommand(ColorSensor colorSensor) {
    super(colorSensor::toggleOverrideSensor, colorSensor);
  }
}
