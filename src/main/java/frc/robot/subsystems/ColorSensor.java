package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TCS34725;
import frc.robot.util.TCS34725.GainSettings;
import java.util.Arrays;
import java.util.List;

/** This class contains all the logic for ball color detection, and comparison to alliance color */
public class ColorSensor extends SubsystemBase {
  // Color thresholds
  private static final List<Double> BLUE_THRESHOLD = Arrays.asList(50.0, 100.0, 50.0);
  private static final List<Double> RED_THRESHOLD = Arrays.asList(100.0, 50.0, 50.0);

  // Instance of TCS34725
  private final TCS34725 tcs34725 = new TCS34725(Port.kOnboard);

  // Alliance color enum
  private enum AllianceColor {
    BLUE,
    RED,
    INVALID
  };

  // Alliance color sendable chooser
  private final SendableChooser<AllianceColor> allianceColorChooser = new SendableChooser<>();

  // Ball color enum
  private enum BallColor {
    BLUE,
    RED,
    UNDETERMINED
  }

  // Ball color
  private BallColor ballColor = BallColor.UNDETERMINED;

  // Override sensor flag
  private boolean overrideSensor = false;

  /** Constructor for the ColorSensor class */
  public ColorSensor() {
    // Set the gain value of the TCS34725
    tcs34725.setGain(GainSettings.ONE_TIMES);

    // Add the options to the alliance color chooser
    allianceColorChooser.addOption("Blue", AllianceColor.BLUE);
    allianceColorChooser.addOption("Red", AllianceColor.RED);

    // Try to automatically get our alliance color from the Driver Station
    if (DriverStation.getAlliance().name().equals("Blue")) {
      // If the Driver Station returns blue set the alliance color chooser default to blue
      allianceColorChooser.setDefaultOption("Blue", AllianceColor.BLUE);
    } else if (DriverStation.getAlliance().name().equals("Red")) {
      // If the Driver Station returns red set the alliance color chooser default to red
      allianceColorChooser.setDefaultOption("Red", AllianceColor.RED);
    } else {
      // If the Driver Station returns invalid set the alliance color chooser default to invalid
      allianceColorChooser.setDefaultOption("INVALID", AllianceColor.INVALID);

      // Report a warning that the alliance color could not be automatically determined
      DriverStation.reportWarning("Could not get alliance color, please set it!", true);
    }

    // Put the alliance color chooser out to the SmartDashboard
    SmartDashboard.putData(allianceColorChooser);
  }

  /** Runs periodically and contains the logic for determining ball color */
  @Override
  public void periodic() {
    // Get the RGB values
    List<Double> rgb = tcs34725.getRGB();

    // Put the RGB values out to SmartDashboard
    SmartDashboard.putNumber("Red", rgb.get(0));
    SmartDashboard.putNumber("Green", rgb.get(1));
    SmartDashboard.putNumber("Blue", rgb.get(2));

    // Determine ball color
    if (rgb.get(0) <= BLUE_THRESHOLD.get(0)
        && rgb.get(1) <= BLUE_THRESHOLD.get(1)
        && rgb.get(2) >= BLUE_THRESHOLD.get(2)) {
      this.ballColor = BallColor.BLUE;
    } else if (rgb.get(0) >= RED_THRESHOLD.get(0)
        && rgb.get(1) <= RED_THRESHOLD.get(1)
        && rgb.get(2) <= RED_THRESHOLD.get(2)) {
      this.ballColor = BallColor.RED;
    } else {
      this.ballColor = BallColor.UNDETERMINED;
    }

    // Put the ball color out to SmartDashboard
    SmartDashboard.putString("Ball Color", ballColor.name());

    // Put override sensor flag out to SmartDashboard
    SmartDashboard.putBoolean("Color Sensor Override", overrideSensor);
  }

  /** Returns whether the detected ball is the wrong color */
  public boolean isWrongColor() {
    // Check if the override sensor flag is set
    if (overrideSensor) {
      // Put Is Wrong Color as false to SmartDashboard
      SmartDashboard.putBoolean("Is Wrong Color", false);

      // If it is return false
      return false;
    }

    // Check our alliance color
    if (allianceColorChooser.getSelected() == AllianceColor.BLUE) {
      // If our alliance color is blue and the detected ball is red return true
      if (ballColor == BallColor.RED) {
        // Put Is Wrong Color as true to SmartDashboard
        SmartDashboard.putBoolean("Is Wrong Color", true);

        // If the ball is red return true
        return true;
      }
    } else if (allianceColorChooser.getSelected() == AllianceColor.RED) {
      // If our alliance color is red and the detected ball is blue return true
      if (ballColor == BallColor.BLUE) {
        // Put Is Wrong Color as true to SmartDashboard
        SmartDashboard.putBoolean("Is Wrong Color", true);

        // If the ball is blue return true
        return true;
      }
    } else {
      // If our alliance color is invalid report a warning to the Driver Station
      DriverStation.reportWarning("Could not get alliance color, please set it!", true);
    }

    // Put Is Wrong Color as false to SmartDashboard
    SmartDashboard.putBoolean("Is Wrong Color", false);

    // Return false if an incorrect ball was not detected
    return false;
  }

  /** Toggles the override sensor flag */
  public void toggleOverrideSensor() {
    // Set the override sensor flag to the opposite of its current value
    this.overrideSensor = !this.overrideSensor;
  }
}
