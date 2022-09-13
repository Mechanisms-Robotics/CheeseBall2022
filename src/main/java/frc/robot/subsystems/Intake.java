package frc.robot.subsystems;

import static frc.robot.Constants.startupCanTimeout;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** This class contains all the code that controls the intake functionality */
public class Intake extends SubsystemBase {
  // Intake speeds
  private static final double INTAKE_SPEED = 0.5; // percent
  private static final double OUTTAKE_SPEED = -0.25; // percent

  // Intake actuator positions
  private static final DoubleSolenoid.Value INTAKE_DEPLOYED = Value.kReverse;
  private static final DoubleSolenoid.Value INTAKE_RETRACTED = Value.kForward;

  // Intake motor
  private final WPI_TalonFX intakeMotor = new WPI_TalonFX(20);

  // Intake actuator
  private final DoubleSolenoid intakeActuator =
      new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 2);

  // Intake motor configuration
  private static final TalonFXConfiguration INTAKE_MOTOR_CONFIGURATION = new TalonFXConfiguration();

  // Configure the intake current limit
  static {
    // Instantiate a new SupplyCurrentLimitConfiguration
    final var intakeCurrentLimit = new SupplyCurrentLimitConfiguration();

    // Configure the settings of the SupplyCurrentLimitConfiguration
    intakeCurrentLimit.currentLimit = 15; // amps
    intakeCurrentLimit.triggerThresholdCurrent = 18; // amps
    intakeCurrentLimit.triggerThresholdTime = 0.25; // sec
    intakeCurrentLimit.enable = true;

    // Set the INTAKE_MOTOR_CONFIGURATION current limit and disable reverse and forward limits
    INTAKE_MOTOR_CONFIGURATION.supplyCurrLimit = intakeCurrentLimit;
    INTAKE_MOTOR_CONFIGURATION.reverseSoftLimitEnable = false;
    INTAKE_MOTOR_CONFIGURATION.forwardSoftLimitEnable = false;
  }

  /** Constructor for the Intake class */
  public Intake() {
    // Configure intake motor
    intakeMotor.configAllSettings(INTAKE_MOTOR_CONFIGURATION, startupCanTimeout);
    intakeMotor.setInverted(TalonFXInvertType.Clockwise);
    intakeMotor.setNeutralMode(NeutralMode.Coast);

    // CAN bus utilization optimization
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    intakeMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
  }

  /** Deploys the intake */
  public void deploy() {
    // Set the intake actuator to the INTAKE_DEPLOYED position
    intakeActuator.set(INTAKE_DEPLOYED);
  }

  /** Retracts the intake */
  public void retract() {
    // Set the intake actuator to the INTAKE_RETRACTED position
    intakeActuator.set(INTAKE_RETRACTED);
  }

  /** Runs the intake */
  public void intake() {
    // Set the intake motor to run at INTAKE_SPEED
    intakeMotor.set(ControlMode.PercentOutput, INTAKE_SPEED);
  }

  /** Runs the intake in reverse */
  public void outtake() {
    // Set the intake motor to run at OUTTAKE_SPEED
    intakeMotor.set(ControlMode.PercentOutput, OUTTAKE_SPEED);
  }

  /** Stops the intake */
  public void stop() {
    // Set the intake motor to 0% power
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }

  /** Returns whether the intake is deployed or not */
  public boolean isDeployed() {
    // Return if the intake actuator is in the INTAKE_DEPLOYED position
    return intakeActuator.get() == INTAKE_DEPLOYED;
  }

  /** Returns whether the intake is retracted or not */
  public boolean isRetracted() {
    // Return the opposite of what isDeployed returns
    return !isDeployed();
  }
}
