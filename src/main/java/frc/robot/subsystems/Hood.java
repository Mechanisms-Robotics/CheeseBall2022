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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Utils;
import frc.robot.util.Units;

/** This class contains all the code that controls the hood functionality */
public class Hood extends SubsystemBase {
  // Hood constants
  private static final double HOOD_GEAR_RATIO =
      (480.0 / 30.0)
          * (52.0 / 24.0)
          * (56.0 / 24.0)
          * (52.0 / 24.0)
          * (36.0 / 18.0); // 350.52:1 reduction
  private static final double HOOD_FORWARD_LIMIT = Math.toRadians(29.0); // 29 degrees
  private static final double HOOD_REVERSE_LIMIT = Math.toRadians(0.0); // 0 degrees
  private static final double HOOD_ALLOWABLE_ERROR = Math.toDegrees(0.5); // 0.5 degrees
  private static final double HOOD_ANGLE_LOBF_SLOPE = 3.38;
  private static final double HOOD_ANGLE_LOBF_INTERCEPT = 14.61;
  private static final double HOOD_ERROR_EPSILON = Math.toRadians(1.0); // 1 degree

  // Hood motor
  private final WPI_TalonFX hoodMotor = new WPI_TalonFX(70);

  // Hood motor configuration
  private static final TalonFXConfiguration HOOD_MOTOR_CONFIGURATION = new TalonFXConfiguration();

  // Configure hood motor current limit and PID
  static {
    // Configure hood motor current limit
    final var hoodCurrentLimit = new SupplyCurrentLimitConfiguration();
    hoodCurrentLimit.currentLimit = 10; // amps
    hoodCurrentLimit.triggerThresholdCurrent = 15; // amps
    hoodCurrentLimit.triggerThresholdTime = 0.5; // sec
    hoodCurrentLimit.enable = true;
    HOOD_MOTOR_CONFIGURATION.supplyCurrLimit = hoodCurrentLimit;

    // Hood motor soft limits
    HOOD_MOTOR_CONFIGURATION.forwardSoftLimitThreshold =
        Units.radsToFalcon(HOOD_FORWARD_LIMIT, HOOD_GEAR_RATIO);
    HOOD_MOTOR_CONFIGURATION.reverseSoftLimitThreshold =
        Units.radsToFalcon(HOOD_REVERSE_LIMIT, HOOD_GEAR_RATIO);
    HOOD_MOTOR_CONFIGURATION.forwardSoftLimitEnable = true;
    HOOD_MOTOR_CONFIGURATION.reverseSoftLimitEnable = true;

    // Hood PID configuration
    final var hoodPositionPID = new SlotConfiguration();
    hoodPositionPID.kP = 0.0;
    hoodPositionPID.allowableClosedloopError =
        Units.radsToFalcon(HOOD_ALLOWABLE_ERROR, HOOD_GEAR_RATIO);
    HOOD_MOTOR_CONFIGURATION.slot0 = hoodPositionPID;

    // Configure hood motor velocity measurement
    HOOD_MOTOR_CONFIGURATION.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_2Ms;
    HOOD_MOTOR_CONFIGURATION.velocityMeasurementWindow = 4;
  }

  // Hood zeroed flag
  private boolean zeroed = false;

  // Desired angle
  private double desiredAngle = 0.0;

  /** Constructor for the Hood class */
  public Hood() {
    // Configure hood motor
    hoodMotor.configAllSettings(HOOD_MOTOR_CONFIGURATION, startupCanTimeout);
    hoodMotor.setInverted(TalonFXInvertType.Clockwise);
    hoodMotor.setNeutralMode(NeutralMode.Brake);
    hoodMotor.selectProfileSlot(0, 0);

    // CAN bus utilization optimization
    hoodMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
  }

  /** Runs periodically and puts the current and desired hood angle on the SmartDashboard */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Angle", Math.toDegrees(getAngle()));
    SmartDashboard.putNumber("Desired Hood Angle", Math.toDegrees(desiredAngle));
  }

  /** Aims the hood at a calculated angle based on the range to the target */
  public void aim(double range) {
    // Check if the turret has been zeroed
    if (!this.zeroed) {
      // If it hasn't, return
      return;
    }

    // Set desiredAngle to the clamped value
    this.desiredAngle =
        MathUtil.clamp(
            Math.toRadians(HOOD_ANGLE_LOBF_SLOPE * range + HOOD_ANGLE_LOBF_INTERCEPT),
            HOOD_REVERSE_LIMIT,
            HOOD_FORWARD_LIMIT);

    // PID the hood motor to the desired position
    hoodMotor.set(ControlMode.Position, Units.radsToFalcon(this.desiredAngle, HOOD_GEAR_RATIO));
  }

  /** Returns whether the hood is within HOOD_ERROR_EPSILON degrees of it's desired angle */
  public boolean atDesiredAngle() {
    return Utils.epsilonEquals(getAngle(), this.desiredAngle, HOOD_ERROR_EPSILON);
  }

  /** Gets the current angle of the hood in rads */
  public double getAngle() {
    // Return the current angle of the hood in radians
    return Units.falconToRads(hoodMotor.getSelectedSensorPosition(), HOOD_GEAR_RATIO);
  }

  /** Zeros the hood, allowing it to be run */
  public void zero() {
    // Check if the hood has already been zeroed
    if (this.zeroed) {
      // If it has, return
      return;
    }

    // Zero the hood motor encoder and set the zeroed flag to true
    hoodMotor.setSelectedSensorPosition(0.0);
    this.zeroed = true;
  }
}
