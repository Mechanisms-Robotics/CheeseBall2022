// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.feeder.FeederRunCommand;
import frc.robot.commands.intake.IntakeDeployCommand;
import frc.robot.commands.intake.IntakeRetractCommand;
import frc.robot.commands.intake.IntakeRunCommand;
import frc.robot.commands.processor.ProcessorRunCommand;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;
import frc.robot.util.ControllerWrapper;
import frc.robot.util.StartEndScheduleCommand;

public class RobotContainer {

	private final Intake intake = new Intake();
	private final Processor processor = new Processor();
	private final Feeder feeder = new Feeder();

	private final ControllerWrapper controller = new ControllerWrapper(0);

	private final Button intakeDeployButton = new Button(controller::getLeftBumperButton);
	private final Button intakeRetractButton = new Button(controller::getRightBumperButton);
	private final Button intakeRunButton = new Button(controller::getTriangleButton);

	public RobotContainer() {
		configureButtonBindings();

	}

	private void configureButtonBindings() {

		intakeDeployButton.whenPressed(new IntakeDeployCommand(intake));

		intakeRetractButton.whenPressed(new IntakeRetractCommand(intake));

		intakeRunButton.toggleWhenPressed(new ParallelCommandGroup(
				new IntakeRunCommand(intake),
				new ProcessorRunCommand(processor),
				new FeederRunCommand(feeder)
		));

	}

	public Command getAutonomousCommand() {
		return null;
	}
}
