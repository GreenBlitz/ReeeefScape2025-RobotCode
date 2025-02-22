package frc;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.field.enums.Cage;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.subsystems.climb.ClimbState;
import frc.robot.subsystems.climb.ClimbStateHandler;
import frc.robot.subsystems.climb.lifter.LifterState;
import frc.robot.subsystems.climb.lifter.LifterStateHandler;
import frc.robot.subsystems.climb.solenoid.SolenoidState;
import frc.robot.subsystems.climb.solenoid.SolenoidStateHandler;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.utils.pose.Side;
import frc.utils.utilcommands.ExecuteEndCommand;

public class JoysticksBindings {

	private static final double NOTE_IN_RUMBLE_TIME_SECONDS = 0.5;
	private static final double NOTE_IN_RUMBLE_POWER = 0.4;

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

		Trigger noteIn = new Trigger(robot.getRobotCommander().getSuperstructure()::isCoralIn);
		noteIn.onTrue(noteInRumble(MAIN_JOYSTICK).alongWith(noteInRumble(SECOND_JOYSTICK)));

		Trigger noteOut = new Trigger(() -> !robot.getRobotCommander().getSuperstructure().isCoralIn());
		noteOut.onTrue(noteInRumble(MAIN_JOYSTICK).alongWith(noteInRumble(SECOND_JOYSTICK)));
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

	private static Command noteInRumble(SmartJoystick joystick) {
		return new ExecuteEndCommand(
			() -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, NOTE_IN_RUMBLE_POWER),
			() -> joystick.stopRumble(GenericHID.RumbleType.kBothRumble)
		).withTimeout(NOTE_IN_RUMBLE_TIME_SECONDS);
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...

		usedJoystick.R1.onTrue(robot.getRobotCommander().autoScore());
		usedJoystick.L1.onTrue(robot.getRobotCommander().setState(RobotState.INTAKE));
		usedJoystick.A.onTrue(robot.getRobotCommander().setState(RobotState.DRIVE));
		usedJoystick.B.onTrue(robot.getRobotCommander().setState(RobotState.ALGAE_REMOVE));
		usedJoystick.X.onTrue(robot.getRobotCommander().setState(RobotState.ALGAE_OUTTAKE));

		usedJoystick.POV_UP.onTrue(
			new InstantCommand(
				() -> robot.getRobotCommander()
					.getSuperstructure().driverIsAlgaeInOverride = !robot.getRobotCommander().getSuperstructure().driverIsAlgaeInOverride

			)
		);
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...

		usedJoystick.A.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L1));
		usedJoystick.B.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L2));
		usedJoystick.X.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L3));
		usedJoystick.Y.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L4));

		usedJoystick.R1.onTrue(new InstantCommand(ScoringHelpers::toggleIsLeftBranch));
		usedJoystick.L1.onTrue(new InstantCommand(ScoringHelpers::toggleIsFarReefHalf));

		usedJoystick.POV_UP.onTrue(new InstantCommand(() -> ScoringHelpers.setTargetSideForReef(Side.MIDDLE)));
		usedJoystick.POV_DOWN.onTrue(new InstantCommand(() -> ScoringHelpers.setTargetSideForReef(Side.MIDDLE)));
		usedJoystick.POV_LEFT.onTrue(new InstantCommand(() -> ScoringHelpers.setTargetSideForReef(Side.LEFT)));
		usedJoystick.POV_RIGHT.onTrue(new InstantCommand(() -> ScoringHelpers.setTargetSideForReef(Side.RIGHT)));
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
//		robot.getSwerve().applyCalibrationBindings(usedJoystick, () -> robot.getPoseEstimator().getEstimatedPose());

		SolenoidStateHandler solenoidStateHandler = new SolenoidStateHandler(robot.getSolenoid());
		LifterStateHandler lifterStateHandler = new LifterStateHandler(robot.getLifter());

		//RobotCommander - FULL CLIMB testing
		usedJoystick.L3.onTrue(robot.getRobotCommander().setState(RobotState.PRE_CLIMB));
		usedJoystick.R3.onTrue(robot.getRobotCommander().setState(RobotState.CLIMB));
		
		// Climb State Testing

		ClimbStateHandler climbStateHandler = new ClimbStateHandler(solenoidStateHandler, lifterStateHandler);

		usedJoystick.A.onTrue(climbStateHandler.setState(ClimbState.STOP));
		usedJoystick.B.onTrue(climbStateHandler.setState(ClimbState.DEPLOY));
		usedJoystick.X.onTrue(climbStateHandler.setState(ClimbState.CLIMB));

		// Manual Climbing for Lifter Position
		usedJoystick.POV_DOWN.onTrue(solenoidStateHandler.setState(SolenoidState.INITIAL_FREE));
		usedJoystick.POV_RIGHT.onTrue(solenoidStateHandler.setState(SolenoidState.HOLD_FREE));
		usedJoystick.POV_LEFT.onTrue(solenoidStateHandler.setState(SolenoidState.LOCKED));

		usedJoystick.L1.onTrue(lifterStateHandler.setState(LifterState.DEPLOY));
		usedJoystick.R1.onTrue(lifterStateHandler.setState(LifterState.CLIMB));
		usedJoystick.Y.onTrue(lifterStateHandler.setState(LifterState.HOLD));

		// Climb aim assist testing
		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER)
			.onTrue(robot.getSwerve().getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE));

		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER)
			.onTrue(robot.getSwerve().getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CAGE)));

		usedJoystick.POV_UP.onTrue(new InstantCommand(() -> ScoringHelpers.targetCage = Cage.FIELD_WALL));
		usedJoystick.BACK.onTrue(new InstantCommand(() -> ScoringHelpers.targetCage = Cage.MIDDLE));
		usedJoystick.START.onTrue(new InstantCommand(() -> ScoringHelpers.targetCage = Cage.FIELD_CENTER));
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...
		robot.getElevator().applyCalibrationBindings(usedJoystick);
	}

	private static void fifthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...
//		robot.getArm().applyCalibrationBindings(usedJoystick);

		robot.getSolenoid().applyCalibrationBindings(usedJoystick);
		robot.getLifter().applyCalibrationBindings(usedJoystick);
	}

	private static void sixthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SIXTH_JOYSTICK;

		robot.getEndEffector().applyCalibrationsBindings(usedJoystick);
	}

}
