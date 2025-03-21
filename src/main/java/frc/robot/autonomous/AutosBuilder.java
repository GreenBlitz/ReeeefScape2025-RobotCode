package frc.robot.autonomous;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.utils.auto.AutoPath;
import frc.utils.auto.PathHelper;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class AutosBuilder {

	public static List<Supplier<PathPlannerAutoWrapper>> getAllTestAutos() {
		return List.of(
			() -> new PathPlannerAutoWrapper("Rotate"),
			() -> new PathPlannerAutoWrapper("Rotate 2m"),
			() -> new PathPlannerAutoWrapper("Straight 2m")
		);
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllStartingAndScoringFirstObjectAutos(
		Robot robot,
		Supplier<Command> scoringCommand,
		Pose2d tolerance
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (AutoPath autoPath : PathHelper.getAllStartingAndScoringFirstObjectPaths()) {
			autos.add(
				() -> createAutoFromAutoPath(
					autoPath,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.commandAfterPath(robot, pathPlannerPath, scoringCommand, autoPath.getTargetBranch(), tolerance)
				)
			);
		}
		return autos;
	}

//	public static List<Supplier<Command>> getAllPreBuiltAutos(
//		Robot robot,
//		Supplier<Command> intakingCommand,
//		Supplier<Command> scoringCommand,
//		Pose2d tolerance
//	) {
//		ArrayList<Supplier<Command>> autos = new ArrayList<>();
//		autos.add(() -> preBuiltLeftAuto(robot, intakingCommand, scoringCommand, tolerance));
//		autos.add(() -> preBuiltCenterAuto(robot));
//		autos.add(() -> preBuiltRightAuto(robot, intakingCommand, scoringCommand, tolerance));
//		return autos;
//	}

	public static List<Supplier<Command>> getAllNoDelayAutos(
		Robot robot,
		Supplier<Command> intakingCommand,
		Supplier<Command> scoringCommand,
		Pose2d tolerance
	) {
		ArrayList<Supplier<Command>> autos = new ArrayList<>();
		autos.add(() -> leftNoDelayAuto(robot, intakingCommand, scoringCommand, tolerance));
		autos.add(() -> centerNoDelayAuto(robot));
		autos.add(() -> rightNoDelayAuto(robot, intakingCommand, scoringCommand, tolerance));
		return autos;
	}

//	public static List<Supplier<PathPlannerAutoWrapper>> getAllAutoScoringAutos(Robot robot) {
//		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
//		for (Branch branch : Branch.values()) {
//			autos.add(() -> autoScoreToChosenBranch(branch, robot));
//		}
//		return autos;
//	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllIntakingAutos(Robot robot, Supplier<Command> intakingCommand, Pose2d tolerance) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (AutoPath autoPath : PathHelper.getAllIntakingPaths()) {
			autos.add(
				() -> createAutoFromAutoPath(
					autoPath,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.deadlinePathWithCommand(robot, pathPlannerPath, intakingCommand, autoPath.getTargetBranch(), tolerance)
				)
			);
		}
		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllScoringAutos(Robot robot, Supplier<Command> scoringCommand, Pose2d tolerance) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (AutoPath autoPath : PathHelper.getAllScoringPathsFromCoralStations()) {
			autos.add(
				() -> createAutoFromAutoPath(
					autoPath,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.commandAfterPath(robot, pathPlannerPath, scoringCommand, autoPath.getTargetBranch(), tolerance)
				)
			);
		}
		return autos;
	}

	public static PathPlannerAutoWrapper autoScoreToBranch(Branch branch, Robot robot, PathPlannerPath path) {
		return new PathPlannerAutoWrapper(new InstantCommand(() -> {
			ScoringHelpers.targetScoreLevel = ScoreLevel.L4;
			ScoringHelpers.isLeftBranch = branch.isLeft();
			ScoringHelpers.isFarReefHalf = branch.getReefSide().isFar();
			ScoringHelpers.setTargetSideForReef(branch.getReefSide().getSide());
		}).andThen(robot.getRobotCommander().autoScoreForAutonomous(path)), Pose2d.kZero, branch.name() + " Auto Score", true);
	}

	public static Command autoScoreToChosenBranch(Robot robot, PathPlannerPath path) {
		return robot.getRobotCommander().autoScoreForAutonomous(path);
	}

	public static PathPlannerPath getAutoScorePath(Branch branch, Robot robot) {
		ScoringHelpers.targetScoreLevel = ScoreLevel.L4;
		ScoringHelpers.setTargetBranch(branch);

		Pose2d startingPose = robot.getPoseEstimator().getEstimatedPose();
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
			AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
			new IdealStartingState(0, startingPose.getRotation()),
			new GoalEndState(0, scoringPose.getRotation()),
			false
		);
		path.preventFlipping = true;
		Logger.recordOutput("Auto/FirstPath", path.getPathPoses().toArray(new Pose2d[] {}));
		return path;
	}

	public static PathPlannerAutoWrapper createAutoFromAutoPath(AutoPath path, Function<PathPlannerPath, Command> pathFollowingCommand) {
		Optional<PathPlannerPath> pathOptional = path.getPath();

		return new PathPlannerAutoWrapper(
			pathOptional.map(pathFollowingCommand).orElse(Commands.none()),
			pathOptional.map(PathPlannerUtil::getPathStartingPose).orElse(path.getStartingPoint().getSecond()),
			path.getPathName(),
			pathOptional.isPresent()
		);
	}

	public static Command leftNoDelayAuto(Robot robot, Supplier<Command> intakingCommand, Supplier<Command> scoringCommand, Pose2d tolerance) {
		PathPlannerPath path = getAutoScorePath(Branch.I, robot);

		Command auto = new SequentialCommandGroup(
			autoScoreToChosenBranch(robot, path),
			new SequentialCommandGroup(
				createAutoFromAutoPath(
					AutoPath.I_TO_UPPER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.I_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.L)),
				createAutoFromAutoPath(
					AutoPath.UPPER_CORAL_STATION_2_TO_L,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.herm(robot, pathPlannerPath, scoringCommand, AutoPath.UPPER_CORAL_STATION_2_TO_L.getTargetBranch(), tolerance)
				),
				createAutoFromAutoPath(
					AutoPath.L_TO_UPPER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.L_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.K)),
				createAutoFromAutoPath(
					AutoPath.UPPER_CORAL_STATION_2_TO_K,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.herm(robot, pathPlannerPath, scoringCommand, AutoPath.UPPER_CORAL_STATION_2_TO_K.getTargetBranch(), tolerance)
				),
				createAutoFromAutoPath(
					AutoPath.K_TO_UPPER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.K_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.A)),
				createAutoFromAutoPath(
					AutoPath.UPPER_CORAL_STATION_2_TO_A,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.herm(robot, pathPlannerPath, scoringCommand, AutoPath.UPPER_CORAL_STATION_2_TO_A.getTargetBranch(), tolerance)
				)
			).asProxy()
		);
		auto.setName("left no delay");
		return auto;
	}

	private static Command rightNoDelayAuto(Robot robot, Supplier<Command> intakingCommand, Supplier<Command> scoringCommand, Pose2d tolerance) {
		PathPlannerPath path = getAutoScorePath(Branch.F, robot);

		Command auto = new SequentialCommandGroup(
			autoScoreToChosenBranch(robot, path),
			new SequentialCommandGroup(
				createAutoFromAutoPath(
					AutoPath.F_TO_LOWER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.E_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				createAutoFromAutoPath(
					AutoPath.LOWER_CORAL_STATION_2_TO_C,
					pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
						robot,
						pathPlannerPath,
						scoringCommand,
						AutoPath.LOWER_CORAL_STATION_2_TO_C.getTargetBranch(),
						tolerance
					)
				),
				createAutoFromAutoPath(
					AutoPath.C_TO_LOWER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.C_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				createAutoFromAutoPath(
					AutoPath.LOWER_CORAL_STATION_2_TO_D,
					pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
						robot,
						pathPlannerPath,
						scoringCommand,
						AutoPath.LOWER_CORAL_STATION_2_TO_D.getTargetBranch(),
						tolerance
					)
				),
				createAutoFromAutoPath(
					AutoPath.D_TO_LOWER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.D_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				createAutoFromAutoPath(
					AutoPath.LOWER_CORAL_STATION_2_TO_E,
					pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
						robot,
						pathPlannerPath,
						scoringCommand,
						AutoPath.LOWER_CORAL_STATION_2_TO_E.getTargetBranch(),
						tolerance
					)
				)
			).asProxy()
		);
		auto.setName("right no delay");
		return auto;
	}

	private static Command centerNoDelayAuto(Robot robot) {
		PathPlannerPath path = getAutoScorePath(Branch.H, robot);

		Command auto = autoScoreToChosenBranch(robot, path);
		auto.setName("center no delay");
		return auto;
	}


	public static Command createDefaultNoDelayAuto(Robot robot) {
		ChassisPowers chassisPowers = new ChassisPowers();
		chassisPowers.xPower = AutonomousConstants.DEFAULT_AUTO_DRIVE_POWER;

		Command auto = new ParallelCommandGroup(
			robot.getSwerve()
				.getCommandsBuilder()
				.drive(() -> chassisPowers)
				.withTimeout(AutonomousConstants.DEFAULT_AUTO_DRIVE_TIME_SECONDS)
				.andThen(robot.getSwerve().getCommandsBuilder().resetTargetSpeeds()),
			robot.getRobotCommander().getSuperstructure().elevatorOpening()
		);
		auto.setName("default no delay");
		return auto;
	}

//	private static PathPlannerAutoWrapper preBuiltRightAuto(
//		Robot robot,
//		Supplier<Command> intakingCommand,
//		Supplier<Command> scoringCommand,
//		Pose2d tolerance
//	) {
//		PathPlannerAutoWrapper auto = PathPlannerAutoWrapper.chainAutos(
//			autoScoreToChosenBranch(Branch.F, robot),
//			PathPlannerAutoWrapper
//				.chainAutos(
//					createAutoFromAutoPath(
//						AutoPath.F_TO_LOWER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.F_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.LOWER_CORAL_STATION_2_TO_C,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.LOWER_CORAL_STATION_2_TO_C.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.C_TO_LOWER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.C_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.LOWER_CORAL_STATION_2_TO_D,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.LOWER_CORAL_STATION_2_TO_D.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.D_TO_LOWER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.D_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.LOWER_CORAL_STATION_2_TO_E,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.LOWER_CORAL_STATION_2_TO_E.getTargetBranch(),
//							tolerance
//						)
//					)
//				)
//				.asProxyAuto()
//		);
//		auto.setName("right");
//		return auto;
//	}
//
//	private static PathPlannerAutoWrapper preBuiltCenterAuto(Robot robot) {
//		PathPlannerAutoWrapper auto = autoScoreToChosenBranch(Branch.H, robot);
//		auto.setName("center");
//		return auto;
//	}
//
//	private static PathPlannerAutoWrapper preBuiltLeftAuto(
//		Robot robot,
//		Supplier<Command> intakingCommand,
//		Supplier<Command> scoringCommand,
//		Pose2d tolerance
//	) {
//		PathPlannerAutoWrapper auto = PathPlannerAutoWrapper.chainAutos(
//			autoScoreToChosenBranch(Branch.I, robot),
//			PathPlannerAutoWrapper
//				.chainAutos(
//					createAutoFromAutoPath(
//						AutoPath.I_TO_UPPER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.I_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.UPPER_CORAL_STATION_2_TO_L,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.UPPER_CORAL_STATION_2_TO_L.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.L_TO_UPPER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.L_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.UPPER_CORAL_STATION_2_TO_K,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.UPPER_CORAL_STATION_2_TO_K.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.K_TO_UPPER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.K_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.UPPER_CORAL_STATION_2_TO_J,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.UPPER_CORAL_STATION_2_TO_J.getTargetBranch(),
//							tolerance
//						)
//					)
//				)
//				.asProxyAuto()
//		);
//		auto.setName("left");
//		return auto;
//	}
//
//	public static PathPlannerAutoWrapper createDefaultAuto(Robot robot) {
//		return new PathPlannerAutoWrapper(
//			new ParallelCommandGroup(
//				robot.getSwerve()
//					.getCommandsBuilder()
//					.drive(() -> new ChassisPowers(AutonomousConstants.DEFAULT_AUTO_DRIVE_POWER, 0, 0))
//					.withTimeout(AutonomousConstants.DEFAULT_AUTO_DRIVE_TIME_SECONDS)
//					.andThen(robot.getSwerve().getCommandsBuilder().resetTargetSpeeds()),
//				robot.getRobotCommander().getSuperstructure().elevatorOpening()
//			),
//			Pose2d.kZero,
//			"DefaultAuto",
//			true
//		);
//	}

}
