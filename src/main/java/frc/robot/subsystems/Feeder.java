package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

	private static final TalonFXConfiguration FEEDER_CONFIG = new TalonFXConfiguration();
	private final WPI_TalonFX feederMotor = new WPI_TalonFX(40);

	private static final double FEEDER_SPEED = 0.4;

	static {
		// TODO: motor config
	}

	public Feeder() {
	}

	public void on() {
		setWheelOpenLoop(FEEDER_SPEED);
	}

	public void off() {
		setWheelOpenLoop(0.0);
	}

	public void setWheelOpenLoop(double percentOutput) {
		feederMotor.set(ControlMode.PercentOutput, percentOutput);
	}
}
