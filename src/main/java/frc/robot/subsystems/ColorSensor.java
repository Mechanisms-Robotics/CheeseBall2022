package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class contains all the logic for ball color detection, and comparison to alliance color */
public class ColorSensor extends SubsystemBase {
  // DIO Sensor Readings
  private static final DigitalInput redDetected = new DigitalInput(3);
  private static final DigitalInput blueDetected = new DigitalInput(4);

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
    // Determine ball color
    if (blueDetected.get() && !redDetected.get()) {
      this.ballColor = BallColor.BLUE;
    } else if (redDetected.get() && !blueDetected.get()) {
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
