package frc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.enums.Branch;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.Swerve;


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

	public static void setDriversInputsToSwerve(Swerve swerve) {
		if (MAIN_JOYSTICK.isConnected()) {
			swerve.setDriversPowerInputs(
				new ChassisPowers(
					MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y),
					MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X),
					MAIN_JOYSTICK.getAxisValue(Axis.RIGHT_X)
				)
			);
		} else if (THIRD_JOYSTICK.isConnected()) {
			swerve.setDriversPowerInputs(
				new ChassisPowers(
					THIRD_JOYSTICK.getAxisValue(Axis.LEFT_Y),
					THIRD_JOYSTICK.getAxisValue(Axis.LEFT_X),
					THIRD_JOYSTICK.getAxisValue(Axis.RIGHT_X)
				)
			);
		} else {
			swerve.setDriversPowerInputs(new ChassisPowers(0, 0, 0));
		}
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
		usedJoystick.A.onTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> new Pose2d(2, 2, Rotation2d.fromDegrees(60)))
		);

		usedJoystick.X.onTrue(robot.getRobotCommander().scoreWithMoveToPose(ScoreLevel.L4, Branch.C));
		usedJoystick.Y.onTrue(robot.getRobotCommander().scoreWithMoveToPose(ScoreLevel.L3, Branch.F));
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
