package frc.robot.statemachine;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.autonomous.PathFollowingCommandsBuilder;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.scoringhelpers.ScoringPathsHelper;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.utils.pose.PoseUtil;

import java.util.Set;
import java.util.function.Supplier;

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
		this.currentState = RobotState.STAY_IN_PLACE;
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

	public void initializeDefaultCommand() {
		setDefaultCommand(
			new DeferredCommand(
				() -> endState(currentState),
				Set.of(
					this,
					superstructure,
					swerve,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid()
				)
			)
		);
	}

	/**
	 * Check if robot at pose but relative to target branch. Y-axis is vertical to the branch. X-axis is horizontal to the branch So when you
	 * check if robot in place in y-axis its in parallel to the reef side.
	 */
	private boolean isAtReefScoringPose(
		double scoringPoseDistanceFromReefMeters,
		Pose2d l1Tolerances,
		Pose2d l1Deadbands,
		Pose2d tolerances,
		Pose2d deadbands
	) {
		Rotation2d reefAngle = Field.getReefSideMiddle(ScoringHelpers.getTargetBranch().getReefSide()).getRotation();

		Pose2d reefRelativeTargetPose = ScoringHelpers
			.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), scoringPoseDistanceFromReefMeters)
			.rotateBy(reefAngle.unaryMinus());
		Pose2d reefRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(reefAngle.unaryMinus());

		ChassisSpeeds allianceRelativeSpeeds = swerve.getAllianceRelativeVelocity();
		ChassisSpeeds reefRelativeSpeeds = SwerveMath
			.robotToAllianceRelativeSpeeds(allianceRelativeSpeeds, Field.getAllianceRelative(reefAngle.unaryMinus()));

		return switch (ScoringHelpers.targetScoreLevel) {
			case L1 -> PoseUtil.isAtPose(reefRelativeRobotPose, reefRelativeTargetPose, reefRelativeSpeeds, l1Tolerances, l1Deadbands);
			case L2, L3, L4 -> PoseUtil.isAtPose(reefRelativeRobotPose, reefRelativeTargetPose, reefRelativeSpeeds, tolerances, deadbands);
		};
	}

	private boolean isAtBranchScoringPose(
		Branch targetBranch,
		double scoringPoseDistanceFromReefMeters,
		Pose2d l1Tolerances,
		Pose2d l1Deadbands,
		Pose2d tolerances,
		Pose2d deadbands
	) {
		Rotation2d reefAngle = Field.getReefSideMiddle(targetBranch.getReefSide()).getRotation();

		Pose2d reefRelativeTargetPose = ScoringHelpers.getRobotBranchScoringPose(targetBranch, scoringPoseDistanceFromReefMeters)
			.rotateBy(reefAngle.unaryMinus());
		Pose2d reefRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(reefAngle.unaryMinus());

		ChassisSpeeds allianceRelativeSpeeds = swerve.getAllianceRelativeVelocity();
		ChassisSpeeds reefRelativeSpeeds = SwerveMath
			.robotToAllianceRelativeSpeeds(allianceRelativeSpeeds, Field.getAllianceRelative(reefAngle.unaryMinus()));

		return switch (ScoringHelpers.targetScoreLevel) {
			case L1 -> PoseUtil.isAtPose(reefRelativeRobotPose, reefRelativeTargetPose, reefRelativeSpeeds, l1Tolerances, l1Deadbands);
			case L2, L3, L4 -> PoseUtil.isAtPose(reefRelativeRobotPose, reefRelativeTargetPose, reefRelativeSpeeds, tolerances, deadbands);
		};
	}

	private boolean isAtProcessorScoringPose() {
		Rotation2d processorAngle = Field.getProcessor().getRotation();

		Pose2d processorRelativeTargetPose = ScoringHelpers.getAllianceRelativeProcessorScoringPose().rotateBy(processorAngle.unaryMinus());
		Pose2d processorRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(processorAngle.unaryMinus());

		ChassisSpeeds allianceRelativeSpeeds = swerve.getAllianceRelativeVelocity();
		ChassisSpeeds processorRelativeSpeeds = SwerveMath
			.robotToAllianceRelativeSpeeds(allianceRelativeSpeeds, Field.getAllianceRelative(processorAngle.unaryMinus()));

		return PoseUtil.isAtPose(
			processorRelativeRobotPose,
			processorRelativeTargetPose,
			processorRelativeSpeeds,
			Tolerances.PROCESSOR_RELATIVE_SCORING_POSITION,
			Tolerances.PROCESSOR_RELATIVE_SCORING_DEADBANDS
		);
	}

	private boolean isReadyToOpenSuperstructure() {
		return isAtReefScoringPose(
			StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS,
			Tolerances.REEF_RELATIVE_L1_OPEN_SUPERSTRUCTURE_POSITION,
			Tolerances.REEF_RELATIVE_L1_OPEN_SUPERSTRUCTURE_DEADBANDS,
			Tolerances.REEF_RELATIVE_OPEN_SUPERSTRUCTURE_POSITION,
			Tolerances.REEF_RELATIVE_OPEN_SUPERSTRUCTURE_DEADBANDS
		);
	}

	private boolean isPreScoreReady() {
		return superstructure.isPreScoreReady()
			&& isAtReefScoringPose(
				StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS,
				Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS,
				Tolerances.REEF_RELATIVE_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
			);
	}

	/**
	 * Checks if the robot is out of the safe zone to close the superstructure
	 */
	public boolean isReadyToCloseSuperstructure() {
		Rotation2d reefAngle = Field.getReefSideMiddle(ScoringHelpers.getTargetReefSide()).getRotation();

		Translation2d reefRelativeReefSideMiddle = Field.getReefSideMiddle(ScoringHelpers.getTargetReefSide())
			.rotateBy(reefAngle.unaryMinus())
			.getTranslation();
		Translation2d reefRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(reefAngle.unaryMinus()).getTranslation();

		return !PoseUtil
			.isAtTranslation(reefRelativeRobotPose, reefRelativeReefSideMiddle, StateMachineConstants.CLOSE_SUPERSTRUCTURE_LENGTH_AND_WIDTH);
	}

	public boolean isReadyToScore() {
		return superstructure.isReadyToScore()
			&& isAtReefScoringPose(
				StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS,
				Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS,
				Tolerances.REEF_RELATIVE_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
			);
	}

	public boolean isAtBranchScoringPose(Branch branch) {
		return isAtBranchScoringPose(
			branch,
			StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS,
			Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
			Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS,
			Tolerances.REEF_RELATIVE_SCORING_POSITION,
			Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
		);
	}

	public boolean isReadyToActivateCoralStationAimAssist() {
		Translation2d robotTranslation = robot.getPoseEstimator().getEstimatedPose().getTranslation();
		Translation2d coralStationSlotTranslation = Field.getCoralStationSlot(ScoringHelpers.getTargetCoralStationSlot(robot)).getTranslation();
		return robotTranslation.getDistance(coralStationSlotTranslation)
			<= StateMachineConstants.DISTANCE_FROM_CORAL_STATION_SLOT_TO_START_AIM_ASSIST_METERS;
	}

	private boolean isCloseToNet(double distanceOnXAxis, double distanceOnYAxis) {
		boolean isPastX = Field.getAllianceRelative(robot.getPoseEstimator().getEstimatedPose().getTranslation(), true, true).getX()
			> Field.LENGTH_METERS / 2 - distanceOnXAxis;
		boolean isPastY = Field.getAllianceRelative(robot.getPoseEstimator().getEstimatedPose().getTranslation(), true, true).getY()
			> Field.WIDTH_METERS / 2 - distanceOnYAxis;
		return isPastX && isPastY;
	}

	public boolean isReadyForNet() {
		return isCloseToNet(
			StateMachineConstants.SCORE_DISTANCES_FROM_MIDDLE_OF_BARGE_METRES.getX(),
			StateMachineConstants.SCORE_DISTANCES_FROM_MIDDLE_OF_BARGE_METRES.getY()
		) && swerve.isAtHeading(ScoringHelpers.getHeadingForNet(), Tolerances.HEADING_FOR_NET, Tolerances.HEADING_FOR_NET_DEADBAND);
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case DRIVE -> drive();
			case STAY_IN_PLACE -> stayInPlace();
			case INTAKE_WITH_AIM_ASSIST -> intakeWithAimAssist();
			case INTAKE_WITHOUT_AIM_ASSIST -> intakeWithoutAimAssist();
			case CORAL_OUTTAKE -> coralOuttake();
			case ALIGN_REEF -> alignReef();
			case ARM_PRE_SCORE -> armPreScore();
			case PRE_SCORE -> preScore();
			case SCORE_WITHOUT_RELEASE -> scoreWithoutRelease();
			case SCORE -> score();
			case ALGAE_REMOVE -> algaeRemove();
			case ALGAE_OUTTAKE -> algaeOuttake();
			case PRE_NET -> preNet();
			case NET -> net();
			case PROCESSOR_SCORE -> fullyProcessorScore();
			case PRE_CLIMB_WITH_AIM_ASSIST -> preClimbWithAimAssist();
			case PRE_CLIMB_WITHOUT_AIM_ASSIST -> preClimbWithoutAimAssist();
			case CLIMB -> climb();
			case STOP_CLIMB -> stopClimb();
			case CLOSE_CLIMB -> closeClimb();
		};
	}

	public Command autoScore() {
		Supplier<Command> fullySuperstructureScore = () -> new SequentialCommandGroup(
			superstructure.armPreScore().until(this::isReadyToOpenSuperstructure),
			superstructure.preScore().until(superstructure::isPreScoreReady),
			superstructure.scoreWithoutRelease().until(this::isReadyToScore),
			superstructure.scoreWithRelease()
		);

		Supplier<Command> driveToPath = () -> swerve.getCommandsBuilder()
			.driveToPath(
				() -> robot.getPoseEstimator().getEstimatedPose(),
				ScoringPathsHelper.getPathByBranch(ScoringHelpers.getTargetBranch()),
				ScoringHelpers
					.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
			);

		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelDeadlineGroup(fullySuperstructureScore.get(), driveToPath.get()),
				Set.of(
					this,
					superstructure,
					swerve,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid()
				)
			),
			RobotState.SCORE
		);
	}

	public Command autoScoreForAutonomous(PathPlannerPath path) {
		Supplier<Command> fullySuperstructureScore = () -> new SequentialCommandGroup(
			superstructure.elevatorOpening(),
			superstructure.armPreScore().until(this::isReadyToOpenSuperstructure),
			superstructure.preScore().until(superstructure::isPreScoreReady),
			superstructure.scoreWithoutRelease().until(this::isReadyToScore),
			superstructure.scoreWithRelease()
		);

		Supplier<Command> driveToPath = () -> PathFollowingCommandsBuilder.followPath(path)
			.andThen(
				swerve.getCommandsBuilder()
					.moveToPoseByPID(
						() -> robot.getPoseEstimator().getEstimatedPose(),
						ScoringHelpers.getRobotBranchScoringPose(
							ScoringHelpers.getTargetBranch(),
							StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS
						)
					)
			);

		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelDeadlineGroup(fullySuperstructureScore.get(), driveToPath.get()),
				Set.of(
					this,
					superstructure,
					swerve,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid()
				)
			),
			RobotState.SCORE
		);
	}

	public Command fullyScore() {
		return new SequentialCommandGroup(
			armPreScore().until(this::isReadyToOpenSuperstructure),
			preScore().until(this::isPreScoreReady),
			scoreWithoutRelease().until(this::isReadyToScore),
			score()
		);
	}

	public Command scoreForButton() {
		return new SequentialCommandGroup(scoreWithoutRelease().until(this::isReadyToScore), score());
	}

	public Command fullyPreScore() {
		return new SequentialCommandGroup(
			armPreScore().until(this::isReadyToOpenSuperstructure),
			preScore().until(this::isPreScoreReady),
			scoreWithoutRelease()
		);
	}


	public Command fullyProcessorScore() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				new SequentialCommandGroup(superstructure.idle().until(this::isAtProcessorScoringPose), superstructure.processorScore()),
				swerve.getCommandsBuilder()
					.driveToPose(robot.getPoseEstimator()::getEstimatedPose, ScoringHelpers::getAllianceRelativeProcessorScoringPose)
			),
			RobotState.PROCESSOR_SCORE
		);
	}

	public Command fullyNet() {
		return asSubsystemCommand(new SequentialCommandGroup(preNet().until(this::isReadyForNet), net()), RobotState.NET);
	}

	private Command drive() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.idle(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.DRIVE
		);
	}

	private Command stayInPlace() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.stayInPlace(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.STAY_IN_PLACE
		);
	}

	private Command intakeWithAimAssist() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.intake(),
				new SequentialCommandGroup(
					swerve.getCommandsBuilder()
						.driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CORAL_STATION))
						.until(this::isReadyToActivateCoralStationAimAssist),
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CORAL_STATION_SLOT))
				)
			),
			RobotState.INTAKE_WITH_AIM_ASSIST
		);
	}

	private Command intakeWithoutAimAssist() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(superstructure.intake(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.INTAKE_WITHOUT_AIM_ASSIST
		);
	}

	private Command coralOuttake() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(superstructure.outtake(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.CORAL_OUTTAKE
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

	private Command armPreScore() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.armPreScore(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.ARM_PRE_SCORE
		);
	}

	private Command preScore() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preScore(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.PRE_SCORE
		);
	}

	private Command scoreWithoutRelease() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.scoreWithoutRelease(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.SCORE_WITHOUT_RELEASE
		);
	}

	private Command score() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.scoreWithRelease(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.SCORE
		);
	}

	private Command algaeRemove() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.algaeRemove(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.ALGAE_REMOVE))
			),
			RobotState.ALGAE_REMOVE
		);
	}

	private Command algaeOuttake() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.algaeOuttake(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
			),
			RobotState.ALGAE_OUTTAKE
		);
	}

	private Command preNet() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.idle(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NET))
			),
			RobotState.PRE_NET
		);
	}

	private Command net() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.netWithRelease(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NET))
			),
			RobotState.NET
		);
	}

	private Command preClimbWithAimAssist() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preClimb(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CAGE_ROTATION))
			),
			RobotState.PRE_CLIMB_WITH_AIM_ASSIST
		);
	}

	private Command preClimbWithoutAimAssist() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.preClimb(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.PRE_CLIMB_WITHOUT_AIM_ASSIST
		);
	}

	private Command climb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.climb(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.CLIMB
		);
	}

	public Command stopClimb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.climbStop(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.STOP_CLIMB
		);
	}

	public Command closeClimb() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(superstructure.closeClimb(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.CLOSE_CLIMB
		);
	}

	private Command closeAfterScore() {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(
				new ParallelCommandGroup(
					superstructure.afterScore(),
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
				).until(this::isReadyToCloseSuperstructure),
				drive()
			),
			Set.of(
				this,
				superstructure,
				swerve,
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid()
			)
		);
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case STAY_IN_PLACE, CORAL_OUTTAKE -> stayInPlace();
			case
				INTAKE_WITH_AIM_ASSIST,
				INTAKE_WITHOUT_AIM_ASSIST,
				DRIVE,
				ALIGN_REEF,
				ALGAE_REMOVE,
				ALGAE_OUTTAKE,
				PROCESSOR_SCORE,
				PRE_NET,
				NET ->
				drive();
			case ARM_PRE_SCORE, CLOSE_CLIMB -> armPreScore();
			case PRE_SCORE -> preScore();
			case SCORE, SCORE_WITHOUT_RELEASE -> closeAfterScore();
			case PRE_CLIMB_WITH_AIM_ASSIST -> preClimbWithAimAssist();
			case PRE_CLIMB_WITHOUT_AIM_ASSIST -> preClimbWithoutAimAssist();
			case CLIMB, STOP_CLIMB -> stopClimb();
		};
	}

}
