package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Units;

/** This class contains all the code that controls the turret functionality */
public class Turret extends SubsystemBase {
  // Turret constants
  private static final double TURRET_GEAR_RATIO =
      (225.0 / 30.0) * (56.0 / 30.0) * (64.0 / 30.0) * (56.0 / 30.0); // 55.75:1 reduction

  private static final double TURRET_FORWARD_LIMIT = Math.toRadians(90.0); // 90 degrees
  private static final double TURRET_REVERSE_LIMIT = Math.toRadians(-90.0); // -90 degrees
  private static final double TURRET_ALLOWABLE_ERROR = Math.toRadians(0.5); // 0.5 degrees

  // Turret motor
  private final WPI_TalonFX turretMotor = new WPI_TalonFX(50);

  // Turret motor configuration
  private static final TalonFXConfiguration TURRET_MOTOR_CONFIGURATION = new TalonFXConfiguration();

  // Configure the turret current limit and PID
  static {
    // Current limit configuration for the turret motor
    final var turretCurrentLimit = new SupplyCurrentLimitConfiguration();
    turretCurrentLimit.currentLimit = 10; // amps
    turretCurrentLimit.triggerThresholdCurrent = 15; // amps
    turretCurrentLimit.triggerThresholdTime = 0.5; // sec
    turretCurrentLimit.enable = true;
    TURRET_MOTOR_CONFIGURATION.supplyCurrLimit = turretCurrentLimit;

    // Turret motor soft limits
    TURRET_MOTOR_CONFIGURATION.forwardSoftLimitThreshold =
        Units.radsToFalcon(TURRET_FORWARD_LIMIT, TURRET_GEAR_RATIO);
    TURRET_MOTOR_CONFIGURATION.reverseSoftLimitThreshold =
        Units.radsToFalcon(TURRET_REVERSE_LIMIT, TURRET_GEAR_RATIO);
    TURRET_MOTOR_CONFIGURATION.forwardSoftLimitEnable = true;
    TURRET_MOTOR_CONFIGURATION.reverseSoftLimitEnable = true;

    // Turret PID configuration
    final var turretPositionPID = new SlotConfiguration();
    turretPositionPID.kP = 0.0;
    turretPositionPID.allowableClosedloopError =
        Units.radsToFalcon(TURRET_ALLOWABLE_ERROR, TURRET_GEAR_RATIO);
    TURRET_MOTOR_CONFIGURATION.slot0 = turretPositionPID;

    // Turret motor velocity measurement configuration
    TURRET_MOTOR_CONFIGURATION.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_2Ms;
    TURRET_MOTOR_CONFIGURATION.velocityMeasurementWindow = 4;
  }

  // Zeroed flag
  private boolean zeroed = false;

  // Desired angle
  private double desiredAngle = 0.0;

  /** Constructor for the Turret class */
  public Turret() {
    // Configure intake motor
    turretMotor.configAllSettings(TURRET_MOTOR_CONFIGURATION, startupCanTimeout);
    turretMotor.setInverted(TalonFXInvertType.Clockwise);
    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.selectProfileSlot(0, 0);

    // CAN bus utilization optimization
    turretMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
  }

  /** Aims the turret at some desired angle */
  public void aim(double rads) {
    // Check if the turret has been zeroed
    if (!this.zeroed) {
      // If it hasn't, return
      return;
    }

    // Set desiredAngle to the clamped value
    this.desiredAngle = MathUtil.clamp(rads, TURRET_REVERSE_LIMIT, TURRET_FORWARD_LIMIT);

    // PID the turret motor to the desired position
    turretMotor.set(ControlMode.Position, Units.radsToFalcon(this.desiredAngle, TURRET_GEAR_RATIO));
  }

  /** Gets the current angle of the turret in rads */
  public double getAngle() {
    // Return the current angle of the turret in radians
    return Units.falconToRads(turretMotor.getSelectedSensorPosition(), TURRET_GEAR_RATIO);
  }

  /** Zeros the turret, allowing it to be run */
  public void zero() {
    // Check if the turret has already been zeroed
    if (this.zeroed) {
      // If it has, return
      return;
    }

    // Zero the turret motor encoder and set the zeroed flag to true
    turretMotor.setSelectedSensorPosition(0.0);
    this.zeroed = true;
  }
}
