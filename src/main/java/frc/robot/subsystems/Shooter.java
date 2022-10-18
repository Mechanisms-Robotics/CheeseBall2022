package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Utils;
import frc.robot.util.Units;

/** This class contains all the code that controls the shooter functionality */
public class Shooter extends SubsystemBase {
  // Shooter constants
  private static final double SHOOTER_GEAR_RATIO = (64.0 / 42.0); // 1.52:1 reduction
  private static final double SHOOTER_RPM_LOBF_SLOPE = 123.11;
  private static final double SHOOTER_RPM_LOBF_INTERCEPT = 1080.82;
  private static final double SHOOTER_ERROR_EPSILON = 10; // RPM
  private static final double TUNING_SCALAR = 1.22;

  // Shooter motors
  private final WPI_TalonFX shooterMotor = new WPI_TalonFX(60);
  private final WPI_TalonFX shooterFollowerMotor = new WPI_TalonFX(61);

  // Shooter motors configurations
  private static final TalonFXConfiguration SHOOTER_MOTOR_CONFIGURATION =
      new TalonFXConfiguration();

  // Configure shooter motors current limit and PID
  static {
    // Configure shooter motors current limit
    final var shooterCurrentLimit = new SupplyCurrentLimitConfiguration();
    shooterCurrentLimit.currentLimit = 40; // Amps
    shooterCurrentLimit.triggerThresholdCurrent = 45; // Amps
    shooterCurrentLimit.triggerThresholdTime = 0.5; // sec
    shooterCurrentLimit.enable = true;
    SHOOTER_MOTOR_CONFIGURATION.supplyCurrLimit = shooterCurrentLimit;

    // Shooter motors PID configuration
    final var shooterPID = new SlotConfiguration();
    shooterPID.kP = 0.15;
    shooterPID.kF = 0.055;
    SHOOTER_MOTOR_CONFIGURATION.slot0 = shooterPID;

    // Configure shooter motors velocity measurement
    SHOOTER_MOTOR_CONFIGURATION.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_2Ms;
    SHOOTER_MOTOR_CONFIGURATION.velocityMeasurementWindow = 4;
  }

  // Desired RPM
  double desiredRPM = 0.0;

  /** Constructor for the Shooter class */
  public Shooter() {
    // Configure shooter motor
    shooterMotor.configAllSettings(SHOOTER_MOTOR_CONFIGURATION, startupCanTimeout);
    shooterMotor.setInverted(TalonFXInvertType.Clockwise);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.selectProfileSlot(0, 0);

    // Configure shooter follower motor
    shooterFollowerMotor.configAllSettings(SHOOTER_MOTOR_CONFIGURATION, startupCanTimeout);
    shooterFollowerMotor.follow(shooterMotor);
    shooterFollowerMotor.setInverted(InvertType.OpposeMaster);
    shooterFollowerMotor.setNeutralMode(NeutralMode.Coast);

    // CAN bus utilization optimisation
    shooterMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    shooterMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

    shooterFollowerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    shooterFollowerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
  }

  /** Runs periodically and puts the current shooter RPM on the SmartDashboard */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter RPM", getRPM());
  }

  /** Runs the shooter at a calculated RPM given a range to the target */
  public void shoot(double range) {
    // Calculate and set the desired RPM
    this.desiredRPM = SHOOTER_RPM_LOBF_SLOPE * range + SHOOTER_RPM_LOBF_INTERCEPT;

    // Run shooter motor at the calculated velocity
    shooterMotor.set(
        ControlMode.Velocity,
        Units.RPMToFalcon(this.desiredRPM, SHOOTER_GEAR_RATIO) * TUNING_SCALAR);
    shooterFollowerMotor.set(TalonFXControlMode.Follower, 60);
  }

  /** Returns whether the shooter is within SHOOTER_EPSILON RPM of it's desired speed */
  public boolean atDesiredSpeed() {
    return Utils.epsilonEquals(getRPM(), this.desiredRPM, SHOOTER_ERROR_EPSILON);
  }

  /** Returns the RPM of the shooter */
  private double getRPM() {
    return Units.falconToRPM(shooterMotor.getSelectedSensorVelocity(), SHOOTER_GEAR_RATIO);
  }
}
