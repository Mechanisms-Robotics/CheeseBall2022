// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.intake.IntakeDeployCommand;
import frc.robot.commands.intake.IntakeRetractCommand;
import frc.robot.commands.intake.IntakeRunCommand;
import frc.robot.subsystems.Intake;
import frc.robot.util.ControllerWrapper;

public class RobotContainer {

	private final Intake intake = new Intake();
	private final ControllerWrapper controller = new ControllerWrapper(0);

	private final Button intakeButton = new Button(controller::getLeftBumperButton);
	private final Button intakeRetractButton = new Button(controller::getRightBumperButton);
	private final Button intakeRunButton = new Button(controller::getTriangleButton);

	public RobotContainer() {
		configureButtonBindings();

	}

	private void configureButtonBindings() {
		intakeButton.whenPressed(new SequentialCommandGroup(
				new IntakeDeployCommand(intake)
		));

		intakeRetractButton.whenPressed(new SequentialCommandGroup(
				new IntakeRetractCommand(intake)
		));

		intakeRunButton.whileHeld(new IntakeRunCommand(intake));

	}

	public Command getAutonomousCommand() {
		return null;
	}
}
