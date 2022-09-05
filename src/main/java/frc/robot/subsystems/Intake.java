package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake extends SubsystemBase {

	public enum State {
		Deployed,
		Stowed
	}

	private final static double INTAKE_SPEED = 0.2; // percent

	private static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration();
//	private WPI_TalonFX intakeMotor = new WPI_TalonFX(20);
	private CANSparkMax intakeMotor = new CANSparkMax(21, CANSparkMaxLowLevel.MotorType.kBrushless);
	private DoubleSolenoid leftActuator = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);
	private DoubleSolenoid rightActuator = new DoubleSolenoid(PneumaticsModuleType.REVPH, 15, 3);
	private Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);
	private boolean deployed = false;
	private boolean running = false;

	static {
		// TODO: Intake motor config
	}

	public Intake() {

	}

	private void setWheelOpenLoop(double percentOutput) {
//		intakeMotor.set(ControlMode.PercentOutput, percentOutput);
		intakeMotor.set(percentOutput);
	}

	public void intake() {
		setWheelOpenLoop(INTAKE_SPEED);
		running = true;
	}

	public void deploy() {
		leftActuator.set(Value.kForward);
		rightActuator.set(Value.kForward);
		deployed = true;
		SmartDashboard.putString("state", rightActuator.get().toString());
	}

	public void retract() {
		leftActuator.set(Value.kReverse);
		rightActuator.set(Value.kReverse);
		rightActuator.toggle();
		deployed = false;
		SmartDashboard.putString("state", rightActuator.get().toString());
	}

	public void stop() {
		setWheelOpenLoop(0.0);
		running = false;
	}

	public boolean isDeployed() {
		return deployed;
	}

	public boolean isRunning() {
		return running;
	}
}
