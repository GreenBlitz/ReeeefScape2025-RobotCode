package frc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.function.Function;

public class JoystickBindings {

	public enum BindSet {

		NONE,
		EMPTY,
		SWERVE,
		SECOND,
		TESTING

	}

	public static void configureBindings(SmartJoystick joystick, Robot robot) {}

	private static void bind(
		SmartJoystick joystick,
		Trigger bind,
		BindSet bindSetRequirement,
		Command command,
		Function<Command, Trigger> function
	) {
		function.apply(command).and(() -> joystick.getBindSet() == bindSetRequirement);
	}

	private static void bind(
		SmartJoystick joystick,
		BindSet bindSetRequirement,
		Runnable initRun,
		Runnable executeRun,
		Function<Command, Trigger> function
	) {
		function.apply(new InitExecuteCommand(initRun, executeRun)).and(() -> joystick.getBindSet() == bindSetRequirement);
	}

}
