package frc;

import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;

public class JoysticksBindings {

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	public static void configureBindings(Robot robot) {
		mainJoystickButtons(robot);
		secondJoystickButtons(robot);
		thirdJoystickButtons(robot);
		fourthJoystickButtons(robot);
		fifthJoystickButtons(robot);
		sixthJoystickButtons(robot);
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		robot.getSwerve()
			.setDefaultCommand(
				robot.getSwerve()
					.getCommandsBuilder()
					.drive(
						() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
						() -> usedJoystick.getAxisValue(Axis.LEFT_X),
						() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X)
					)
			);

		usedJoystick.R1.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
					() -> usedJoystick.getAxisValue(Axis.LEFT_X),
					() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
					SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)
				)
		);

		usedJoystick.L1.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
					() -> usedJoystick.getAxisValue(Axis.LEFT_X),
					() -> usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X),
					SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)
				)
		);
		// bindings...
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...
	}

	private static void fifthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...
	}

	private static void sixthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SIXTH_JOYSTICK;
		// bindings...
	}

}
