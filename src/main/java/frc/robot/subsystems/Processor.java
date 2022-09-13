package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class contains all the code that controls the processor functionality */
public class Processor extends SubsystemBase {
  // Processor speeds
  private static final double PROCESSOR_INTAKE_SPEED = 0.5;
  private static final double PROCESSOR_OUTTAKE_SPEED = -0.5;

  // Processor motor
  private final WPI_TalonFX processorMotor = new WPI_TalonFX(30);

  // Processor motor configuration
  private static final TalonFXConfiguration PROCESSOR_MOTOR_CONFIGURATION =
      new TalonFXConfiguration();

  // Configure the processor current limit
  static {
    // Instantiate a new SupplyCurrentLimitConfiguration
    final var processorCurrentLimit = new SupplyCurrentLimitConfiguration();

    // Configure the settings of the SupplyCurrentLimitConfiguration
    processorCurrentLimit.currentLimit = 15; // amps
    processorCurrentLimit.triggerThresholdCurrent = 18; // amps
    processorCurrentLimit.triggerThresholdTime = 0.25; // sec
    processorCurrentLimit.enable = true;

    // Set the PROCESSOR_MOTOR_CONFIGURATION current limit and disable reverse and forward limits
    PROCESSOR_MOTOR_CONFIGURATION.supplyCurrLimit = processorCurrentLimit;
    PROCESSOR_MOTOR_CONFIGURATION.reverseSoftLimitEnable = false;
    PROCESSOR_MOTOR_CONFIGURATION.forwardSoftLimitEnable = false;
  }

  /** Constructor for the Processor class */
  public Processor() {
    // Configure processor motor
    processorMotor.configAllSettings(PROCESSOR_MOTOR_CONFIGURATION, startupCanTimeout);
    processorMotor.setInverted(TalonFXInvertType.Clockwise);
    processorMotor.setNeutralMode(NeutralMode.Coast);

    // CAN bus utilization optimization
    processorMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    processorMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
  }

  /** Runs the processor */
  public void intake() {
    // Set the processor motor to run at PROCESSOR_INTAKE_SPEED
    processorMotor.set(ControlMode.PercentOutput, PROCESSOR_INTAKE_SPEED);
  }

  /** Runs the processor in reverse */
  public void outtake() {
    // Set the processor motor to runt at PROCESSOR_OUTTAKE_SPEED
    processorMotor.set(ControlMode.PercentOutput, PROCESSOR_OUTTAKE_SPEED);
  }

  /** Stops the processor */
  public void stop() {
    // Set the processor motor to 0% power
    processorMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
