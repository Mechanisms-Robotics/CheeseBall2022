package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/** This class contains all the code that controls the feeder functionality */
public class Feeder extends SubsystemBase {
  // Feeder speeds
  private static final double FEEDER_INTAKE_SPEED = 0.25;
  private static final double FEEDER_SHOOT_SPEED = 0.5;

  // Eject time
  private static final double EJECT_TIME = 0.375; // seconds

  // Feeder motor
  private final WPI_TalonFX feederMotor = new WPI_TalonFX(40);

  // Feeder motor configuration
  private static final TalonFXConfiguration FEEDER_MOTOR_CONFIGURATION = new TalonFXConfiguration();

  // Configure the feeder current limit
  static {
    // Instantiate a new SupplyCurrentLimitConfiguration
    final var feederCurrentLimit = new SupplyCurrentLimitConfiguration();

    // Configure the settings of the SupplyCurrentLimitConfiguration
    feederCurrentLimit.currentLimit = 15; // amps
    feederCurrentLimit.triggerThresholdCurrent = 18; // amps
    feederCurrentLimit.triggerThresholdTime = 0.25; // sec
    feederCurrentLimit.enable = true;

    // Set the FEEDER_MOTOR_CONFIGURATION current limit and disable reverse and forward limits
    FEEDER_MOTOR_CONFIGURATION.supplyCurrLimit = feederCurrentLimit;
    FEEDER_MOTOR_CONFIGURATION.reverseSoftLimitEnable = false;
    FEEDER_MOTOR_CONFIGURATION.forwardSoftLimitEnable = false;
  }

  // Proximity sensor value suppliers
  private final Supplier<Boolean> feederBottomSensorSupplier;
  private final Supplier<Boolean> feederTopSensorSupplier;

  // Is ejecting flag
  private boolean isEjecting = false;

  // Eject timer
  private final Timer ejectTimer = new Timer();

  // Done ejecting flag
  private boolean doneEjecting = false;

  /** Constructor for the Feeder class */
  public Feeder(
      Supplier<Boolean> feederBottomSensorSupplier, Supplier<Boolean> feederTopSensorSupplier) {
    // Configure feeder motor
    feederMotor.configAllSettings(FEEDER_MOTOR_CONFIGURATION, startupCanTimeout);
    feederMotor.setInverted(TalonFXInvertType.Clockwise);
    feederMotor.setNeutralMode(NeutralMode.Coast);

    // CAN bus utilization optimization
    feederMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    feederMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);

    // Set suppliers
    this.feederBottomSensorSupplier = feederBottomSensorSupplier;
    this.feederTopSensorSupplier = feederTopSensorSupplier;
  }

  /** Runs the feeder differently depending on which proximity sensors are triggered */
  public void intake() {
    // Get the values of the proximity sensors
    boolean feederTopSensorTriggered = !feederBottomSensorSupplier.get();
    boolean feederBottomSensorTriggered = !feederTopSensorSupplier.get();

    if (feederBottomSensorTriggered && !feederTopSensorTriggered) {
      // Set the feeder motor to run at FEEDER_INTAKE_SPEED
      feederMotor.set(ControlMode.PercentOutput, FEEDER_INTAKE_SPEED);
    } else {
      // Set the feeder motor to run at 0% power
      feederMotor.set(ControlMode.PercentOutput, 0.0);
    }
  }

  /** Runs the feeder at it's shooting speed */
  public void shoot() {
    // Set the feeder motor to run at FEEDER_SHOOT_SPEED
    feederMotor.set(ControlMode.PercentOutput, FEEDER_SHOOT_SPEED);
  }

  /** Runs the feeder for a set time in order to eject a ball */
  public void eject() {
    // Set the feeder motor to run at FEEDER_SHOOT_SPEED
    feederMotor.set(ControlMode.PercentOutput, FEEDER_SHOOT_SPEED);

    // Reset and start the eject timer
    this.ejectTimer.reset();
    this.ejectTimer.start();

    // Set is ejecting flag to true
    this.isEjecting = true;
  }

  /** Resets ejecting flags */
  public void stopEjecting() {
    // Set is ejecting flag to false
    this.isEjecting = false;

    // Set done ejecting flag to false
    this.doneEjecting = false;
  }

  /** Runs periodically and contains the logic for timed ejecting */
  @Override
  public void periodic() {
    // Check if the ejecting flag is set and if EJECT_TIME has elapsed
    if (this.isEjecting && this.ejectTimer.hasElapsed(EJECT_TIME)) {
      // Stop the feeder
      this.stop();

      // Stop the eject timer
      this.ejectTimer.stop();

      // Set done ejecting to true
      this.doneEjecting = true;
    }
  }

  /** Returns whether the feeder is currently ejecting */
  public boolean isEjecting() {
    // Return is ejecting flag
    return this.isEjecting;
  }

  /** Returns whether the feeder is done ejecting */
  public boolean isDoneEjecting() {
    // Return the done ejecting flag
    return this.doneEjecting;
  }

  /** Stops the feeder */
  public void stop() {
    // Set the feeder motor to run at 0% power
    feederMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
