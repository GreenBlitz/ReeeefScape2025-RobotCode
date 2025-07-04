package frc;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.field.Field;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.RobotCommander;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.utilcommands.ExecuteEndCommand;

import java.util.Set;

public class JoysticksBindings {

	private static final double NOTE_IN_RUMBLE_TIME_SECONDS = 0.5;
	private static final double NOTE_IN_RUMBLE_POWER = 0.4;

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
//	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
//	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
//	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
//	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	private static final ChassisPowers driversInputChassisPowers = new ChassisPowers();

	public static void configureBindings(Robot robot) {
		mainJoystickButtons(robot);
		secondJoystickButtons(robot);
		thirdJoystickButtons(robot);
		fourthJoystickButtons(robot);
		fifthJoystickButtons(robot);
		sixthJoystickButtons(robot);

		Trigger noteIn = new Trigger(robot.getRobotCommander().getSuperstructure()::isCoralIn);
		noteIn.onTrue(noteInRumble(MAIN_JOYSTICK).alongWith(noteInRumble(SECOND_JOYSTICK)));

		Trigger noteOut = new Trigger(() -> !robot.getRobotCommander().getSuperstructure().isCoralIn());
		noteOut.onTrue(noteInRumble(MAIN_JOYSTICK).alongWith(noteInRumble(SECOND_JOYSTICK)));
	}

	public static void setDriversInputsToSwerve(Swerve swerve) {
		if (MAIN_JOYSTICK.isConnected()) {
			driversInputChassisPowers.xPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			driversInputChassisPowers.yPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X);
			driversInputChassisPowers.rotationalPower = MAIN_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		}
//		else if (THIRD_JOYSTICK.isConnected()) {
//			swerve.setDriversPowerInputs(
//				new ChassisPowers(
//					THIRD_JOYSTICK.getAxisValue(Axis.LEFT_Y),
//					THIRD_JOYSTICK.getAxisValue(Axis.LEFT_X),
//					THIRD_JOYSTICK.getAxisValue(Axis.RIGHT_X)
//				)
//			);
//		}
		else {
			driversInputChassisPowers.xPower = 0;
			driversInputChassisPowers.yPower = 0;
			driversInputChassisPowers.rotationalPower = 0;
		}
		swerve.setDriversPowerInputs(driversInputChassisPowers);
	}

	private static Command noteInRumble(SmartJoystick joystick) {
		return new ExecuteEndCommand(
			() -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, NOTE_IN_RUMBLE_POWER),
			() -> joystick.stopRumble(GenericHID.RumbleType.kBothRumble)
		).withTimeout(NOTE_IN_RUMBLE_TIME_SECONDS);
	}

	private static Command reefActionChooser(Robot robot) {
		return new DeferredCommand(
			() -> robot.getRobotCommander().getSuperstructure().isCoralIn()
				? Field.isOnBlueSide(robot.getPoseEstimator().getEstimatedPose().getTranslation()) == Field.isFieldConventionAlliance()
					? (ScoringHelpers.isAutoAlgaeRemoveActivated
						? robot.getRobotCommander().autoScoreThenAlgaeRemove()
						: robot.getRobotCommander().autoScore())
					: new InstantCommand()
				: new InstantCommand(() -> ScoringHelpers.setClosetReefSideTarget(robot))
					.andThen(robot.getRobotCommander().setState(RobotState.ALGAE_REMOVE)),
			Set.of(
				robot.getRobotCommander(),
				robot.getRobotCommander().getSuperstructure(),
				robot.getSwerve(),
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid(),
				robot.getPivot(),
				robot.getRollers(),
				robot.getRobotCommander().getLedStateHandler()
			)
		);
	}

	private static Command closeReefActionChooser(Robot robot) {
		return new SequentialCommandGroup(
			new InstantCommand(() -> ScoringHelpers.setClosetReefSideTarget(robot)),
			new DeferredCommand(
				() -> reefActionChooser(robot),
				Set.of(
					robot.getRobotCommander(),
					robot.getRobotCommander().getSuperstructure(),
					robot.getSwerve(),
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getPivot(),
					robot.getRollers(),
					robot.getSolenoid(),
					robot.getRobotCommander().getLedStateHandler()
				)
			)
		);
	}

	private static Command driveActionChooser(Robot robot) {
		return new InstantCommand(() -> {
			RobotCommander robotCommander = robot.getRobotCommander();
			RobotState state = robotCommander.getCurrentState();
			Command command;
			if (state == RobotState.ALGAE_REMOVE || state == RobotState.PRE_NET) {
				robotCommander.setState(RobotState.HOLD_ALGAE).schedule();
				return;
			} else if (state == RobotState.NET) {
				command = robotCommander.driveWith("Soft close net", robotCommander.getSuperstructure().softCloseNet(), true);
			} else if (
				(state == RobotState.SCORE || state == RobotState.SCORE_WITHOUT_RELEASE || state == RobotState.PRE_SCORE)
					&& ScoringHelpers.targetScoreLevel == ScoreLevel.L4
			) {
				command = robotCommander.driveWith("Soft close l4", robotCommander.getSuperstructure().softCloseL4(), true);
			} else {
				command = Commands.none();
			}
			command.andThen(robotCommander.setState(RobotState.DRIVE)).schedule();
		});
	}

	private static Command intakeActionChooser(Robot robot) {
		return new InstantCommand(() -> {
			RobotCommander robotCommander = robot.getRobotCommander();
			RobotState state = robotCommander.getCurrentState();
			Command command;
			if (state == RobotState.NET) {
				command = robotCommander.driveWith("Soft close net", robotCommander.getSuperstructure().softCloseNet(), true);
			} else if (
				(state == RobotState.SCORE || state == RobotState.SCORE_WITHOUT_RELEASE || state == RobotState.PRE_SCORE)
					&& ScoringHelpers.targetScoreLevel == ScoreLevel.L4
			) {
				command = robotCommander.driveWith("Soft close l4", robotCommander.getSuperstructure().softCloseL4(), true);
			} else {
				command = Commands.none();
			}
			command.andThen(robot.getRobotCommander().setState(RobotState.INTAKE_WITHOUT_AIM_ASSIST)).schedule();
		});
	}

	private static Command netActionChooser(Robot robot) {
		return new InstantCommand(() -> {
			RobotCommander robotCommander = robot.getRobotCommander();
			RobotState state = robotCommander.getCurrentState();
			Command command;

			if (state == RobotState.NET || state == RobotState.PRE_NET) {
				command = robotCommander.setState(RobotState.NET);
			} else if (state == RobotState.AUTO_PRE_NET) {
				command = robotCommander.setState(RobotState.PRE_NET);
			} else {
				command = robotCommander.autoNet();
			}
			command.schedule();
		});
	}

	private static Command algaeOuttakeActionChooser(Robot robot) {
		RobotCommander robotCommander = robot.getRobotCommander();

		return new DeferredCommand(
			() -> robotCommander.setState(
				robotCommander.getSuperstructure().isAlgaeInAlgaeIntake()
					? RobotState.ALGAE_OUTTAKE_FROM_INTAKE
					: RobotState.ALGAE_OUTTAKE_FROM_END_EFFECTOR
			),
			Set.of(
				robotCommander,
				robotCommander.getSuperstructure(),
				robot.getSwerve(),
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid(),
				robot.getPivot(),
				robot.getRollers()
			)
		);
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER).onTrue(closeReefActionChooser(robot));

//		usedJoystick.L1.onTrue(robot.getRobotCommander().setState(RobotState.INTAKE_WITH_AIM_ASSIST));
		usedJoystick.L1.onTrue(robot.getRobotCommander().setState(RobotState.ALGAE_INTAKE));

		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER).onTrue(intakeActionChooser(robot));

		usedJoystick.R1.onTrue(netActionChooser(robot));

		usedJoystick.Y.onTrue(robot.getRobotCommander().setState(RobotState.CORAL_OUTTAKE));
		usedJoystick.X.onTrue(algaeOuttakeActionChooser(robot));
		usedJoystick.B.onTrue(robot.getRobotCommander().setState(RobotState.PROCESSOR_SCORE));


//		usedJoystick.POV_LEFT.onTrue(robot.getRobotCommander().setState(RobotState.PRE_CLIMB_WITH_AIM_ASSIST));
		usedJoystick.POV_UP.onTrue(robot.getRobotCommander().setState(RobotState.PRE_CLIMB_WITHOUT_AIM_ASSIST));
		usedJoystick.POV_DOWN.onTrue(robot.getRobotCommander().setState(RobotState.CLIMB_WITH_LIMIT_SWITCH));
		usedJoystick.A.onTrue(driveActionChooser(robot));

		usedJoystick.START.whileTrue(robot.getRobotCommander().setState(RobotState.MANUAL_CLIMB));
		usedJoystick.BACK.whileTrue(robot.getRobotCommander().setState(RobotState.EXIT_CLIMB));
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...

		usedJoystick.POV_RIGHT.onTrue(new InstantCommand(() -> robot.getRobotCommander().keepAlgaeInIntake = false));
		usedJoystick.POV_LEFT.onTrue(new InstantCommand(() -> robot.getRobotCommander().keepAlgaeInIntake = true));

		usedJoystick.A.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L1));
		usedJoystick.B.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L2));
		usedJoystick.X.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L3));
		usedJoystick.Y.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L4));

		usedJoystick.R1.onTrue(new InstantCommand(() -> ScoringHelpers.isLeftBranch = false));
		usedJoystick.L1.onTrue(new InstantCommand(() -> ScoringHelpers.isLeftBranch = true));

		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER).onTrue(robot.getRobotCommander().setState(RobotState.INTAKE_WITHOUT_AIM_ASSIST));
		usedJoystick.POV_UP.onTrue(new InstantCommand(() -> ScoringHelpers.isAutoAlgaeRemoveActivated = true));

		usedJoystick.L3.onTrue(robot.getRobotCommander().setState(RobotState.PRE_CLIMB_WITHOUT_AIM_ASSIST));
	}

	private static void thirdJoystickButtons(Robot robot) {
//		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...

//		robot.getSwerve().applyCalibrationBindings(usedJoystick, () -> robot.getPoseEstimator().getEstimatedPose());
	}

	private static void fourthJoystickButtons(Robot robot) {
//		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...

//		robot.getElevator().applyCalibrationBindings(usedJoystick);
	}

	private static void fifthJoystickButtons(Robot robot) {
//		SmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...

//		robot.getRobotCommander().getSuperstructure().getClimbStateHandler().applyCalibrationBindings(usedJoystick);
//		robot.getRobotCommander().getSuperstructure().getClimbStateHandler().applyCalibrationBindings(usedJoystick);
	}

	private static void sixthJoystickButtons(Robot robot) {
//		SmartJoystick usedJoystick = SIXTH_JOYSTICK;

//		robot.getArm().applyCalibrationBindings(usedJoystick);
//		robot.getEndEffector().applyCalibrationsBindings(usedJoystick);
	}

}
