package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.utils.pose.PoseUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Set;


public class RobotCommander extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final Superstructure superstructure;
	private final RobotTasksManager robotTasksManager;

	private RobotState currentState;

	public RobotCommander(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.superstructure = new Superstructure("StateMachine/Superstructure", robot);
		this.robotTasksManager = new RobotTasksManager(this);
		this.currentState = RobotState.DRIVE;

//		setDefaultCommand(
//				new DeferredCommand(
//						() -> endState(currentState),
//						Set.of(this, superstructure, swerve, robot.getElevator(), robot.getArm(), robot.getEndEffector())
//				)
//		);
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

	public RobotTasksManager getRobotTaskManager() {
		return robotTasksManager;
	}

	/**
	 * Checks if robot is at pose but relative to target branch. Y-axis is vertical to the branch. X-axis is horizontal to the branch So when you
	 * check if robot in place in y-axis its in parallel to the reef side.
	 */

	private boolean isAtPoseByDistanceFromReef(ScoreLevel level, Branch branch, double distanceMeters) {
		Rotation2d reefAngle = Field.getReefSideMiddle(branch.getReefSide()).getRotation();

		Pose2d reefRelativeTargetPose = ScoringHelpers.getRobotBranchScoringPose(branch, distanceMeters).rotateBy(reefAngle.unaryMinus());

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
					Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS,
					swerve.getRobotRelativeAcceleration(),
					Tolerances.ACCELERATION_DEADBAND_METERS_PER_SECOND_SQUARED
				);
			case L2, L3, L4 ->
				PoseUtil.isAtPose(
					reefRelativeRobotPose,
					reefRelativeTargetPose,
					reefRelativeSpeeds,
					Tolerances.REEF_RELATIVE_SCORING_POSITION,
					Tolerances.REEF_RELATIVE_SCORING_DEADBANDS,
					swerve.getRobotRelativeAcceleration(),
					Tolerances.ACCELERATION_DEADBAND_METERS_PER_SECOND_SQUARED
				);
		};
	}

	private boolean isAtScoringDistanceFromReef() {
		return isAtPoseByDistanceFromReef(
			ScoringHelpers.targetScoreLevel,
			ScoringHelpers.getTargetBranch(),
			StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS
		);
	}

	protected boolean isAtOpenSuperstructureDistanceFromReef() {
		return isAtPoseByDistanceFromReef(
			ScoringHelpers.targetScoreLevel,
			ScoringHelpers.getTargetBranch(),
			StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS
		);
	}

	/**
	 * Checks if elevator and arm in place and is robot at pose but relative to target branch. Y-axis is vertical to the branch. X-axis is
	 * horizontal to the branch So when you check if robot in place in y-axis its in parallel to the reef side.
	 */

	public boolean isPreScoreReady(ScoreLevel level, Branch branch) {
		return isAtOpenSuperstructureDistanceFromReef() && superstructure.isPreScoreReady();
	}

	public boolean isReadyToScore() {
		return isAtScoringDistanceFromReef() && superstructure.isReadyToScore();
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case DRIVE -> drive();
			case INTAKE -> intake();
			case OUTTAKE -> outtake();
			case ALIGN_REEF -> alignReef();
			case ARM_PRE_SCORE -> armPreScore();
			case PRE_SCORE -> preScore();
			case SCORE_WITHOUT_RELEASE -> scoreWithoutRelease();
			case SCORE -> score();
		};
	}

	public Command fullyScore() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				armPreScore().until(this::isAtOpenSuperstructureDistanceFromReef),
				preScore().until(() -> isPreScoreReady(ScoringHelpers.targetScoreLevel, ScoringHelpers.getTargetBranch())),
				scoreWithoutRelease().until(this::isReadyToScore),
				score()
			)
		);
	}

	private Command drive() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.idle(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.DRIVE
		);
	}

	private Command intake() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.intake(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CORAL_STATION))
			),
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

	protected Command armPreScore() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.armPreScore(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.ARM_PRE_SCORE
		);
	}

	private Command regularPreScore() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preScore(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.PRE_SCORE
		);
	}

	private Command autoPreScore() {
		Pose2d waitPose = ScoringHelpers
			.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS);

		Command driveToWait = robot.getSwerve()
			.getCommandsBuilder()
			.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> waitPose)
			.until(this::isAtOpenSuperstructureDistanceFromReef);

		return asSubsystemCommand(
			new SequentialCommandGroup(
				driveToWait,
				superstructure.armPreScore()
					.until(
						() -> robot.getArm()
							.isAtPosition(ScoringHelpers.targetScoreLevel.getArmPreScore().getPosition(), Tolerances.ARM_POSITION)
					),

				superstructure.preScore().until(superstructure::isPreScoreReady)
			).until(superstructure::isPreScoreReady),
			ScoringHelpers.targetScoreLevel.name()
		);
	}

	protected Command preScore() {
		return RobotConstants.isScoringAuto ? autoPreScore() : regularPreScore();
	}

	protected Command exitScore() {
		Pose2d waitPose = ScoringHelpers
			.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS);

		Command driveToWait = robot.getSwerve()
			.getCommandsBuilder()
			.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> waitPose)
			.until(this::isAtOpenSuperstructureDistanceFromReef);

		return asSubsystemCommand(
			new SequentialCommandGroup(superstructure.preScore().until(superstructure::isPreScoreReady), driveToWait),
			ScoringHelpers.targetScoreLevel.name()
		);
	}

	protected Command scoreWithoutRelease() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.scoreWithoutRelease(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.SCORE_WITHOUT_RELEASE
		);
	}

	private Command scoreWithRelease() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.scoreWithRelease(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.SCORE
		);
	}

	private Command autoScore() {
		Pose2d targetPose = ScoringHelpers
			.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS);

		Command driveToTarget = robot.getSwerve()
			.getCommandsBuilder()
			.driveToPose(robot.getPoseEstimator()::getEstimatedPose, () -> targetPose)
			.until(this::isAtScoringDistanceFromReef);

		return asSubsystemCommand(
			new SequentialCommandGroup(driveToTarget, superstructure.scoreWithRelease()),
			ScoringHelpers.targetScoreLevel.name()
		);
	}

	public Command score() {
		return RobotConstants.isScoringAuto ? autoScore() : scoreWithRelease();
	}


	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case INTAKE, OUTTAKE, DRIVE, ALIGN_REEF -> drive();
			case ARM_PRE_SCORE -> armPreScore();
			case PRE_SCORE, SCORE, SCORE_WITHOUT_RELEASE -> preScore();
		};
	}

	public Command completeAutoScore() {
		/*
		 * pre-arm score drive to wait open superstructure drive to target score
		 */

		Command driveToWait = new DeferredCommand( () -> swerve.getCommandsBuilder()
			.driveToPose(() -> robot.getPoseEstimator().getEstimatedPose(),
					() -> ScoringHelpers
							.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS)
					)
			.until(this::isAtOpenSuperstructureDistanceFromReef), Set.of(swerve));
		
		Command driveToScore =new DeferredCommand( () ->swerve.getCommandsBuilder()
			.driveToPose(() -> robot.getPoseEstimator().getEstimatedPose(),
					() -> ScoringHelpers
							.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
			)
			.until(this::isAtScoringDistanceFromReef), Set.of(swerve));

		return new SequentialCommandGroup(
				new InstantCommand(() -> Logger.recordOutput("target branch branch", ScoringHelpers
						.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
				)),
			superstructure.armPreScore().until(() -> robot.getArm().isAtPosition(ScoringHelpers.targetScoreLevel.getArmPreScore().getPosition(), Tolerances.ARM_POSITION)),
			superstructure.armPreScore().raceWith(driveToWait),
			superstructure.preScore().raceWith(driveToScore),
			superstructure.scoreWithRelease(),
				exitScore(),
			setState(RobotState.DRIVE)
		);
	}

}
