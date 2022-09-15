package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class contains all the code that controls the shooter functionality */
public class Shooter extends SubsystemBase {
  // Shooter constants
  private static final double SHOOTER_GEAR_RATIO = (64.0 / 42.0); // 1.52:1 reduction

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
    shooterPID.kP = 0.0;
    shooterPID.kF = 0.0;
    SHOOTER_MOTOR_CONFIGURATION.slot0 = shooterPID;

    // Configure shooter motors velocity measurement
    SHOOTER_MOTOR_CONFIGURATION.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_2Ms;
    SHOOTER_MOTOR_CONFIGURATION.velocityMeasurementWindow = 4;
  }

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
}
