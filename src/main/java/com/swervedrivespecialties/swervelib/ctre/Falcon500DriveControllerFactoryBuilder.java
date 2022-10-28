package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import com.swervedrivespecialties.swervelib.SimpleFeedforwardConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;

public final class Falcon500DriveControllerFactoryBuilder {
  private static final double TICKS_PER_ROTATION = 2048.0;

  private static final int CAN_TIMEOUT_MS = 250;
  private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

  private double nominalVoltage = Double.NaN;
  private double currentLimit = Double.NaN;

  private Function<TalonFX, Function<TalonFXConfiguration, Consumer<SimpleFeedforwardConstants>>> configFunction;

  public Falcon500DriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
    this.nominalVoltage = nominalVoltage;
    return this;
  }

  public boolean hasVoltageCompensation() {
    return Double.isFinite(nominalVoltage);
  }

  public DriveControllerFactory<ControllerImplementation, Integer> build() {
    return new FactoryImplementation();
  }

  public Falcon500DriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
    this.currentLimit = currentLimit;
    return this;
  }

  public Falcon500DriveControllerFactoryBuilder withMotorConfigFunction(Function<TalonFX, Function<TalonFXConfiguration, Consumer<SimpleFeedforwardConstants>>> configFunction) {
    this.configFunction = configFunction;
    return this;
  }

  public boolean hasCurrentLimit() {
    return Double.isFinite(currentLimit);
  }

  private class FactoryImplementation
      implements DriveControllerFactory<ControllerImplementation, Integer> {
    @Override
    public ControllerImplementation create(
        Integer driveConfiguration, ModuleConfiguration moduleConfiguration) {
      TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

      double sensorPositionCoefficient =
          Math.PI
              * moduleConfiguration.getWheelDiameter()
              * moduleConfiguration.getDriveReduction()
              / TICKS_PER_ROTATION;
      double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

      if (hasVoltageCompensation()) {
        motorConfiguration.voltageCompSaturation = nominalVoltage;
      }

      if (hasCurrentLimit()) {
        motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
        motorConfiguration.supplyCurrLimit.enable = true;
      }

      TalonFX motor = new TalonFX(driveConfiguration);
      CtreUtils.checkCtreError(
          motor.configAllSettings(motorConfiguration), "Failed to configure Falcon 500");

      if (hasVoltageCompensation()) {
        // Enable voltage compensation
        motor.enableVoltageCompensation(true);
      }

      motor.setNeutralMode(NeutralMode.Brake);

      motor.setInverted(
          moduleConfiguration.isDriveInverted()
              ? TalonFXInvertType.Clockwise
              : TalonFXInvertType.CounterClockwise);
      motor.setSensorPhase(true);

      var feedforwardConstants = new SimpleFeedforwardConstants();
      configFunction.apply(motor).apply(motorConfiguration).accept(feedforwardConstants);

      // Reduce CAN status frame rates
      CtreUtils.checkCtreError(
          motor.setStatusFramePeriod(
              StatusFrameEnhanced.Status_1_General, STATUS_FRAME_GENERAL_PERIOD_MS, CAN_TIMEOUT_MS),
          "Failed to configure Falcon status frame period");

      return new ControllerImplementation(motor, sensorVelocityCoefficient, feedforwardConstants);
    }
  }

  private class ControllerImplementation implements DriveController {
    private final TalonFX motor;
    private final double sensorVelocityCoefficient;
    private final double nominalVoltage =
        hasVoltageCompensation()
            ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage
            : 12.0;

    private final SimpleMotorFeedforward wheelFeedforward;

    private ControllerImplementation(TalonFX motor, double sensorVelocityCoefficient, SimpleFeedforwardConstants feedforwardConstants) {
      this.motor = motor;
      this.sensorVelocityCoefficient = sensorVelocityCoefficient;
      this.wheelFeedforward = new SimpleMotorFeedforward(
              feedforwardConstants.ks,
              feedforwardConstants.kv,
              feedforwardConstants.ka); // 0.319185544, 2.2544, 0.063528
    }

    @Override
    public void setSpeed(double speedMPS) {
      motor.set(
          TalonFXControlMode.Velocity,
          speedMPS,
          DemandType.ArbitraryFeedForward,
          wheelFeedforward.calculate(speedMPS) / 12.0);
    }

    @Override
    public double getVelocity() {
      return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
    }
  }
}
