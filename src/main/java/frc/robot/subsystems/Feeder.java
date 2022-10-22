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
  private static final double FEEDER_INTAKE_SPEED = -0.3;
  private static final double FEEDER_SHOOT_SPEED = -0.5;
  private static final double FEEDER_EJECT_SPEED = -0.15;
  private static final double FEEDER_UNJAM_SPEED = 0.3;

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
  private final Supplier<Boolean> processorSensorSupplier;
  private final Supplier<Boolean> feederBottomSensorSupplier;
  private final Supplier<Boolean> feederTopSensorSupplier;

  // Override sensors flag
  private boolean overrideSensors = false;

  // Amount of balls in the feeder
  private int balls = 0;

  // Top sensor timer
  private boolean topSensorTimerStarted = false;
  private Timer topSensorTimer = new Timer();

  /** Constructor for the Feeder class */
  public Feeder(
      Supplier<Boolean> processorSensorSupplier,
      Supplier<Boolean> feederBottomSensorSupplier,
      Supplier<Boolean> feederTopSensorSupplier) {
    // Configure feeder motor
    feederMotor.configAllSettings(FEEDER_MOTOR_CONFIGURATION, startupCanTimeout);
    feederMotor.setInverted(TalonFXInvertType.Clockwise);
    feederMotor.setNeutralMode(NeutralMode.Brake);

    // CAN bus utilization optimization
    feederMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    feederMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);

    // Set suppliers
    this.processorSensorSupplier = processorSensorSupplier;
    this.feederBottomSensorSupplier = feederBottomSensorSupplier;
    this.feederTopSensorSupplier = feederTopSensorSupplier;
  }

  /** Runs the feeder differently depending on which proximity sensors are triggered */
  public void intake() {
    // Get the values of the proximity sensors
    boolean feederTopSensorTriggered = !feederTopSensorSupplier.get();
    boolean feederBottomSensorTriggered = !feederBottomSensorSupplier.get();
    boolean processorSensorTriggered = !processorSensorSupplier.get();

    if (!processorSensorTriggered && !feederBottomSensorTriggered && !feederTopSensorTriggered) {
      this.balls = 0;
    } else if (processorSensorTriggered
        && !feederBottomSensorTriggered
        && !feederTopSensorTriggered) {
      this.balls = 1;
    } else if (processorSensorTriggered
        && feederBottomSensorTriggered
        && !feederTopSensorTriggered) {
      this.balls = 2;
    } else if (processorSensorTriggered
        && feederBottomSensorTriggered
        && feederTopSensorTriggered) {
      this.balls = 3;
    }

    switch (balls) {
      case 0:
        feederMotor.set(ControlMode.PercentOutput, 0.0);
        break;
      case 1:
        if (!feederBottomSensorTriggered) {
          feederMotor.set(ControlMode.PercentOutput, FEEDER_INTAKE_SPEED);
        } else {
          feederMotor.set(ControlMode.PercentOutput, 0.0);
        }

        break;
      case 2:
        if (!feederTopSensorTriggered) {
          feederMotor.set(ControlMode.PercentOutput, FEEDER_INTAKE_SPEED);
        } else if (!topSensorTimerStarted) {
          topSensorTimer.start();
          topSensorTimerStarted = true;
        } else if (topSensorTimer.hasElapsed(0.125)) {
          feederMotor.set(ControlMode.PercentOutput, 0.0);
          topSensorTimer.stop();
          topSensorTimer.reset();
          topSensorTimerStarted = false;
        }

        break;
    }
  }

  /** Runs the feeder at it's shooting speed */
  public void shoot() {
    // Set the feeder motor to run at FEEDER_SHOOT_SPEED
    feederMotor.set(ControlMode.PercentOutput, FEEDER_SHOOT_SPEED);
  }

  /** Runs the feeder ar it's eject speed */
  public void eject() {
    // Set the feeder motor to run at FEEDER_EJECT_SPEED
    feederMotor.set(ControlMode.PercentOutput, FEEDER_EJECT_SPEED);
  }

  /** Unjams the feeder */
  public void unjam() {
    feederMotor.set(ControlMode.PercentOutput, FEEDER_UNJAM_SPEED);
  }

  /** Stops the feeder */
  public void stop() {
    // Set the feeder motor to run at 0% power
    feederMotor.set(ControlMode.PercentOutput, 0.0);
  }

  /** Toggles the override sensors flag */
  public void toggleOverrideSensors() {
    // Set overrideSensors to the opposite of the current value
    this.overrideSensors = !this.overrideSensors;

    // Set neutral mode
    if (this.overrideSensors) {
      feederMotor.setNeutralMode(NeutralMode.Coast);
    } else {
      feederMotor.setNeutralMode(NeutralMode.Brake);
    }
  }
}
