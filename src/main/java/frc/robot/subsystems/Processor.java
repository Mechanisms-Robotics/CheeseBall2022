package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Processor extends SubsystemBase {

	private final TalonFXConfiguration PROCESSOR_MOTOR = new TalonFXConfiguration();
	private final WPI_TalonFX processorMotor = new WPI_TalonFX(30);

	private static final double PROCESSOR_SPEED = 0.2;

	public Processor() {
	}

	public void on() {
		setWheelOpenLoop(PROCESSOR_SPEED);
	}

	public void off() {
		setWheelOpenLoop(0.0);
	}

	private void setWheelOpenLoop(double percentOutput) {
		processorMotor.set(ControlMode.PercentOutput, percentOutput);
	}

}
