package frc.robot.autonomous;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.auto.PathHelper;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;

public class AutosBuilder {

	public static List<Supplier<PathPlannerAutoWrapper>> getAllTestAutos() {
		return List.of(
			() -> new PathPlannerAutoWrapper("Rotate"),
			() -> new PathPlannerAutoWrapper("Rotate 2m"),
			() -> new PathPlannerAutoWrapper("Straight 2m")
		);
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllPreBuiltAutos(
		Robot robot,
		Supplier<Command> intakingCommand,
		Supplier<Command> scoringCommand
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		autos.add(() -> leftAuto(robot, intakingCommand, scoringCommand));
		autos.add(() -> centerAuto(robot));
		autos.add(() -> rightAuto(robot, intakingCommand, scoringCommand));
		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllStartingAndScoringFirstObjectAutos(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathConstraints pathfindingConstraints,
		Supplier<Command> scoringCommand
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (
			PathPlannerPath path : PathHelper.getAllStartingAndScoringFirstObjectPaths()
				.values()
				.stream()
				.sorted(Comparator.comparing(path -> path.name))
				.toList()
		) {
			autos
				.add(
					() -> new PathPlannerAutoWrapper(
						new InstantCommand(() -> ScoringHelpers.setTargetBranch(PathHelper.getPathTargetBranch(path))).alongWith(
							PathFollowingCommandsBuilder
								.deadlinePathWithCommand(swerve, currentPose, path, pathfindingConstraints, scoringCommand)
						),
						PathPlannerUtil.getPathStartingPose(path),
						path.name
					)
				);
		}
		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllAutoScoringAutos(Robot robot) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (Branch branch : Branch.values()) {
			autos.add(() -> autoScoreToBranch(branch, robot));
		}
		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllIntakingAutos(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathConstraints pathfindingConstraints,
		Supplier<Command> intakingCommand
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (
			PathPlannerPath path : PathHelper.getAllIntakingPaths().values().stream().sorted(Comparator.comparing(path -> path.name)).toList()
		) {
			autos.add(
				() -> new PathPlannerAutoWrapper(
					PathFollowingCommandsBuilder.deadlinePathWithCommand(swerve, currentPose, path, pathfindingConstraints, intakingCommand),
					PathPlannerUtil.getPathStartingPose(path),
					path.name
				)
			);
		}
		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllScoringAutos(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathConstraints pathfindingConstraints,
		Supplier<Command> scoringCommand
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (
			PathPlannerPath path : PathHelper.getAllScoringPathsFromCoralStations()
				.values()
				.stream()
				.sorted(Comparator.comparing(path -> path.name))
				.toList()
		) {
			autos
				.add(
					() -> new PathPlannerAutoWrapper(
						new InstantCommand(() -> ScoringHelpers.setTargetBranch(PathHelper.getPathTargetBranch(path))).alongWith(
							PathFollowingCommandsBuilder
								.deadlinePathWithCommand(swerve, currentPose, path, pathfindingConstraints, scoringCommand)
						),
						PathPlannerUtil.getPathStartingPose(path),
						path.name
					)
				);
		}
		return autos;
	}

	public static PathPlannerAutoWrapper createDefaultAuto(Robot robot) {
		ChassisPowers chassisPowers = new ChassisPowers();
		chassisPowers.xPower = AutonomousConstants.DEFAULT_AUTO_DRIVE_POWER;

		return new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				robot.getSwerve()
					.getCommandsBuilder()
					.drive(() -> chassisPowers)
					.withTimeout(AutonomousConstants.DEFAULT_AUTO_DRIVE_TIME_SECONDS)
					.andThen(robot.getSwerve().getCommandsBuilder().resetTargetSpeeds()),
				robot.getRobotCommander().getSuperstructure().elevatorOpening()
			),
			Pose2d.kZero,
			"Default Auto"
		);
	}

	public static PathPlannerAutoWrapper leftAuto(Robot robot, Supplier<Command> intakingCommand, Supplier<Command> scoringCommand) {
		return new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				autoScoreToBranch(Branch.I, robot),
				new SequentialCommandGroup(
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("I-US2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.L)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("US2-L"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					),
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("L-US2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.K)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("US2-K"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					),
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("K-US2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.A)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("US2-A"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					)
				).asProxy()
			),
			Pose2d.kZero,
			"Left"
		);
	}

	private static PathPlannerAutoWrapper rightAuto(Robot robot, Supplier<Command> intakingCommand, Supplier<Command> scoringCommand) {
		return new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				autoScoreToBranch(Branch.F, robot),
				new SequentialCommandGroup(
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("F-LS2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.C)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("LS2-C"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					),
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("C-LS2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.D)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("LS2-D"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					),
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("D-LS2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.B)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("LS2-B"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					)
				).asProxy()
			),
			Pose2d.kZero,
			"Right"
		);
	}

	private static PathPlannerAutoWrapper centerAuto(Robot robot) {
		return autoScoreToBranch(Branch.H, robot).withAutoName("Center");
	}

	public static PathPlannerAutoWrapper autoScoreToBranch(Branch branch, Robot robot) {
		return new PathPlannerAutoWrapper(
			robot.getRobotCommander()
				.autoScoreForAutonomous(getAutoScorePath(branch, robot.getSwerve(), robot.getPoseEstimator()::getEstimatedPose)),
			Pose2d.kZero,
			branch.name() + " Auto Score"
		);
	}

	public static PathPlannerPath getAutoScorePath(Branch branch, Swerve swerve, Supplier<Pose2d> currentPose) {
		ScoringHelpers.targetScoreLevel = ScoreLevel.L4;
		ScoringHelpers.setTargetBranch(branch);
		Pose2d startingPose = currentPose.get();
		Pose2d openSuperstructurePose = ScoringHelpers
			.getRobotBranchScoringPose(branch, StateMachineConstants.DISTANCE_TO_BRANCH_FOR_STARTING_PATH);
		Pose2d scoringPose = ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS);
		PathPlannerPath path = new PathPlannerPath(
			PathPlannerPath.waypointsFromPoses(startingPose, openSuperstructurePose, scoringPose),
			List.of(),
			List.of(),
			List.of(
				new ConstraintsZone(
					1,
					2,
					new PathConstraints(
						StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND,
						StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_METERS_PER_SECOND_SQUARED,
						StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND.getRadians(),
						StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND_SQUARED.getRadians()
					)
				)
			),
			List.of(),
			AutonomousConstants.getRealTimeConstraintsForAuto(swerve),
			new IdealStartingState(0, startingPose.getRotation()),
			new GoalEndState(0, scoringPose.getRotation()),
			false
		);
		path.preventFlipping = true;
		path.name = branch.name() + " Auto Score";
		Logger.recordOutput(AutonomousConstants.LOG_PATH_PREFIX + "/FirstPath", path.getPathPoses().toArray(Pose2d[]::new));
		return path;
	}

}
