package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class contains all the code that controls the processor functionality */
public class Processor extends SubsystemBase {
  // Processor speeds
  private static final double PROCESSOR_INTAKE_SPEED = 0.5;
  private static final double PROCESSOR_SLOW_INTAKE_SPEED = 0.25;
  private static final double PROCESSOR_SHOOT_SPEED = 0.5;

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

  // Proximity sensors
  private final DigitalInput processorSensor = new DigitalInput(0);
  private final DigitalInput feederBottomSensor = new DigitalInput(1);
  private final DigitalInput feederTopSensor = new DigitalInput(2);

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

  /** Runs periodically and outputs the values of the proximity sensors to SmartDashboard */
  @Override
  public void periodic() {
    boolean processorSensorTriggered = !processorSensor.get();
    boolean feederBottomSensorTriggered = !feederBottomSensor.get();
    boolean feederTopSensorTriggered = !feederTopSensor.get();

    SmartDashboard.putBoolean("Processor Sensor", processorSensorTriggered);
    SmartDashboard.putBoolean("Feeder Bottom Sensor", feederBottomSensorTriggered);
    SmartDashboard.putBoolean("Feeder Top Sensor", feederTopSensorTriggered);
  }

  /** Runs the processor differently depending on which proximity sensors are triggered */
  public void intake() {
    // Get the values of the proximity sensors
    boolean feederTopSensorTriggered = !feederTopSensor.get();
    boolean feederBottomSensorTriggered = !feederBottomSensor.get();
    boolean processorSensorTriggered = !processorSensor.get();

    // Check which sensors are triggered
    if (!(feederTopSensorTriggered && feederBottomSensorTriggered)) {
      // If the robot doesn't have two balls yet run processor at PROCESSOR_INTAKE_SPEED
      processorMotor.set(ControlMode.PercentOutput, PROCESSOR_INTAKE_SPEED);
    } else if (!processorSensorTriggered) {
      // If the robot has two but not three balls yet run processor at PROCESSOR_SLOW_INTAKE_SPEED
      processorMotor.set(ControlMode.PercentOutput, PROCESSOR_SLOW_INTAKE_SPEED);
    } else {
      // If the robot has three balls stop the processor
      processorMotor.set(ControlMode.PercentOutput, 0.0);
    }
  }

  /** Runs the processor at it's shooting speed */
  public void shoot() {
    // Set the processor motor to run at PROCESSOR_SHOOT_SPEED
    processorMotor.set(ControlMode.PercentOutput, PROCESSOR_SHOOT_SPEED);
  }

  /** Stops the processor */
  public void stop() {
    // Set the processor motor to 0% power
    processorMotor.set(ControlMode.PercentOutput, 0.0);
  }
}
