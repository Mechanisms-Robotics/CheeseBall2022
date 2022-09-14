package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class acts as a Superstructure for the intake, processor, and feeder so that we can both
 * intake and shoot at the same time
 */
public class Superstructure extends SubsystemBase {
  // Subsystems
  private final Intake intake;
  private final Processor processor;
  private final Feeder feeder;

  // Flags
  private boolean intaking = false;
  private boolean shooting = false;

  /** Constructor for the Superstructure class */
  public Superstructure(Intake intake, Processor processor, Feeder feeder) {
    // Set the subsystems
    this.intake = intake;
    this.processor = processor;
    this.feeder = feeder;
  }

  /** Deploys and runs the intake, then sets the intaking flag to true */
  public void intake() {
    // Check if the intake is retracted
    if (intake.isRetracted()) {
      // If it is deploy it
      intake.deploy();
    }

    // Run the intake
    intake.intake();

    // Set the intaking flag to true
    this.intaking = true;
  }

  /** Stops and retracts the intake, then sets the intaking flag to false */
  public void stopIntaking() {
    // Stop the intake
    intake.stop();

    // Check if the intake is deployed
    if (intake.isDeployed()) {
      // If it is retract it
      intake.retract();
    }

    // Set the intaking flag to false
    this.intaking = false;
  }

  /** Sets the shooting flag to true */
  public void shoot() {
    // Set the shooting flag to true
    this.shooting = true;
  }

  /** Sets the shooting flag to false */
  public void stopShooting() {
    // Set the shooting flag to false
    this.shooting = false;
  }

  /**
   * Runs periodically and contains the logic for how to handle intaking, shooting, and both
   * simultaneously
   */
  public void periodic() {
    // Check what flags are set
    if (this.intaking && !this.shooting) {
      // If only the intaking flag is set, run the processor and feeder in intake mode
      processor.intake();
      feeder.intake();
    } else if (this.shooting) {
      // If the shooting flag is set, run the processor and feeder in shooting mode
      processor.shoot();
      feeder.shoot();
    } else {
      // If neither are set stop the processor and feeder
      processor.stop();
      feeder.stop();
    }
  }

  /** Returns whether the superstructure is intaking or not */
  public boolean isIntaking() {
    // Return the intaking flag
    return this.intaking;
  }

  /** Returns whether the superstructure is shooting or not */
  public boolean isShooting() {
    // Return the shooting flag
    return this.shooting;
  }
}
