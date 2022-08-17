// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotContainer {

	static class TestBoard extends SubsystemBase {
		WPI_TalonFX testFalcon = new WPI_TalonFX(10);
		CANSparkMax testNeo = new CANSparkMax(20, MotorType.kBrushless);

		public void setOpenLoop(double value) {
			testFalcon.set(ControlMode.PercentOutput, value);
			testNeo.set(value);
		}
	}

  	public RobotContainer() {
    	configureButtonBindings();

		TestBoard testBoard = new TestBoard();
		testBoard.setDefaultCommand(new RunCommand(() -> testBoard.setOpenLoop(0.1f), testBoard));
	}
  	
	private void configureButtonBindings() {}

  	public Command getAutonomousCommand() {
    	return null;
  	}
}
