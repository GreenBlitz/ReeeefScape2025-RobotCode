package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.utils.pose.PoseUtil;

import java.util.Set;

public class RobotCommander extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final Superstructure superstructure;

	private RobotState currentState;

	public RobotCommander(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.superstructure = new Superstructure("StateMachine/Superstructure", robot);
		this.currentState = RobotState.DRIVE;

		setDefaultCommand(new DeferredCommand(() -> endState(currentState), Set.of(this)));
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

	/**
	 * Checks if robot is at pose but relative to target branch. Y-axis is vertical to the branch. X-axis is horizontal to the branch So when you
	 * check if robot in place in y-axis its in parallel to the reef side.
	 */

	private boolean isAtPoseByDistanceFromReef(ScoreLevel level, Branch branch, double distanceMeters) {
		Rotation2d reefAngle = Field.getReefSideMiddle(branch.getReefSide()).getRotation();

		Pose2d reefRelativeTargetPose = ScoringHelpers.getRobotScoringPose(branch, distanceMeters).rotateBy(reefAngle.unaryMinus());
		Pose2d reefRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(reefAngle.unaryMinus());

		ChassisSpeeds allianceRelativeSpeeds = swerve.getAllianceRelativeVelocity();
		ChassisSpeeds reefRelativeSpeeds = SwerveMath
			.robotToAllianceRelativeSpeeds(allianceRelativeSpeeds, Field.getAllianceRelative(reefAngle.unaryMinus()));

		return switch (level) {
			case L1 ->
				PoseUtil.isAtPose(
					reefRelativeRobotPose,
					reefRelativeTargetPose,
					reefRelativeSpeeds,
					Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
					Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS
				);
			case L2, L3, L4 ->
				PoseUtil.isAtPose(
					reefRelativeRobotPose,
					reefRelativeTargetPose,
					reefRelativeSpeeds,
					Tolerances.REEF_RELATIVE_SCORING_POSITION,
					Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
				);
		};
	}

	/**
	 * Checks if elevator and arm in place and is robot at pose but relative to target branch. Y-axis is vertical to the branch. X-axis is
	 * horizontal to the branch So when you check if robot in place in y-axis its in parallel to the reef side.
	 */

	private boolean isPreScoreReady(ScoreLevel level, Branch branch) {
		return isAtPoseByDistanceFromReef(level, branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
			&& superstructure.isPreScoreReady(level);
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case DRIVE -> drive();
			case INTAKE -> intake();
			case OUTTAKE -> outtake();
			case ALIGN_REEF -> alignReef();
			case PRE_L1 -> autoPreScore(ScoreLevel.L1, ScoringHelpers.targetBranch);
			case PRE_L2 -> autoPreScore(ScoreLevel.L2, ScoringHelpers.targetBranch);
			case PRE_L3 -> autoPreScore(ScoreLevel.L3, ScoringHelpers.targetBranch);
			case PRE_L4 -> autoPreScore(ScoreLevel.L4, ScoringHelpers.targetBranch);
			case L1 -> autoScore(ScoreLevel.L1, ScoringHelpers.targetBranch);
			case L2 -> autoScore(ScoreLevel.L2, ScoringHelpers.targetBranch);
			case L3 -> autoScore(ScoreLevel.L3, ScoringHelpers.targetBranch);
			case L4 -> autoScore(ScoreLevel.L4, ScoringHelpers.targetBranch);
		};
	}

	private Command drive() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.idle(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.DRIVE
		);
	}

	private Command intake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.intake(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CORAL_STATION))
			).until(superstructure::isCoralIn),
			RobotState.INTAKE
		);
	}

	private Command outtake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.outtake(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.OUTTAKE
		);
	}

	private Command alignReef() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.idle(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF))
			),
			RobotState.ALIGN_REEF
		);
	}

	private Command genericPreScore(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					superstructure.idle()
						.until(
							() -> isAtPoseByDistanceFromReef(
								scoreLevel,
								ScoringHelpers.targetBranch,
								StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS
							)
						),
					superstructure.preScore(scoreLevel)
				),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			scoreLevel.getRobotPreScore()
		);
	}

	private Command autoPreScore(ScoreLevel scoreLevel, Branch branch) {
		Pose2d waitPose = ScoringHelpers.getRobotScoringPose(branch, StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS);

		Command driveToWait = robot.getSwerve()
			.getCommandsBuilder()
			.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> waitPose)
			.until(() -> isAtPoseByDistanceFromReef(scoreLevel, branch, StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS));

		return asSubsystemCommand(
			new SequentialCommandGroup(driveToWait, superstructure.preScore(scoreLevel).until(() -> superstructure.isPreScoreReady(scoreLevel))),
			scoreLevel.getRobotPreScore()
		);
	}

	private Command preL1(boolean isAuto) {
		return isAuto ? autoPreScore(ScoreLevel.L1, ScoringHelpers.targetBranch) : genericPreScore(ScoreLevel.L1);
	}

	private Command preL2(boolean isAuto) {
		return isAuto ? autoPreScore(ScoreLevel.L2, ScoringHelpers.targetBranch) : genericPreScore(ScoreLevel.L2);
	}

	private Command preL3(boolean isAuto) {
		return isAuto ? autoPreScore(ScoreLevel.L3, ScoringHelpers.targetBranch) : genericPreScore(ScoreLevel.L3);
	}

	private Command preL4(boolean isAuto) {
		return isAuto ? autoPreScore(ScoreLevel.L4, ScoringHelpers.targetBranch) : genericPreScore(ScoreLevel.L4);
	}

	private Command exitScore(ScoreLevel scoreLevel, Branch branch) {
		Pose2d waitPose = ScoringHelpers.getRobotScoringPose(branch, StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS);

		Command driveToWait = robot.getSwerve()
			.getCommandsBuilder()
			.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> waitPose)
			.until(() -> isAtPoseByDistanceFromReef(scoreLevel, branch, StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS));

		return asSubsystemCommand(
			new SequentialCommandGroup(superstructure.preScore(scoreLevel).until(() -> superstructure.isPreScoreReady(scoreLevel)), driveToWait),
			scoreLevel.getRobotPreScore()
		);
	}

	private Command genericScore(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					superstructure.preScore(scoreLevel)
						.until(() -> isPreScoreReady(scoreLevel, ScoringHelpers.targetBranch))
						.until(
							() -> isAtPoseByDistanceFromReef(
								scoreLevel,
								ScoringHelpers.targetBranch,
								StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS
							)
						),
					superstructure.score(scoreLevel)
				),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			).until(superstructure::isCoralOut),
			scoreLevel.getRobotScore()
		);
	}


	private Command autoScore(ScoreLevel scoreLevel, Branch branch) {
		Pose2d targetPose = ScoringHelpers.getRobotScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS);

		Command driveToTarget = robot.getSwerve()
			.getCommandsBuilder()
			.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> targetPose)
			.until(() -> isAtPoseByDistanceFromReef(scoreLevel, branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS));

		return asSubsystemCommand(new SequentialCommandGroup(driveToTarget, superstructure.score(scoreLevel)), scoreLevel.getRobotScore());
	}

	public Command completeAutoScore(ScoreLevel scoreLevel) {
		return new SequentialCommandGroup(
			autoPreScore(scoreLevel, ScoringHelpers.targetBranch),
			autoScore(scoreLevel, ScoringHelpers.targetBranch),
			exitScore(scoreLevel, ScoringHelpers.targetBranch),
			setState(RobotState.DRIVE)
		);
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case INTAKE, OUTTAKE, DRIVE, ALIGN_REEF -> drive();
			case PRE_L1, L1 -> autoPreScore(ScoreLevel.L1, ScoringHelpers.targetBranch);
			case PRE_L2, L2 -> autoPreScore(ScoreLevel.L2, ScoringHelpers.targetBranch);
			case PRE_L3, L3 -> autoPreScore(ScoreLevel.L3, ScoringHelpers.targetBranch);
			case PRE_L4, L4 -> autoPreScore(ScoreLevel.L4, ScoringHelpers.targetBranch);
		};
	}

}
