package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.utils.auto.AutoPath;
import frc.utils.auto.PathHelper;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;

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

	public static List<Supplier<PathPlannerAutoWrapper>> getPreBuiltAutos(
		Robot robot,
		Supplier<Command> scoringCommand,
		Supplier<Command> intakingCommand,
		Pose2d tolerance
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		autos.add(() -> {
			PathPlannerAutoWrapper auto = PathPlannerAutoWrapper.chainAutos(
				createAutoFromAutoPath(
					AutoPath.AUTO_LINE_2_TO_I,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.commandAfterPath(robot, pathPlannerPath, scoringCommand, AutoPath.AUTO_LINE_2_TO_I.getTargetBranch(), tolerance)
				),
				createAutoFromAutoPath(
					AutoPath.I_TO_UPPER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.I_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				createAutoFromAutoPath(
					AutoPath.UPPER_CORAL_STATION_2_TO_L,
					pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
						robot,
						pathPlannerPath,
						scoringCommand,
						AutoPath.UPPER_CORAL_STATION_2_TO_L.getTargetBranch(),
						tolerance
					)
				),
				createAutoFromAutoPath(
					AutoPath.L_TO_UPPER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.L_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				createAutoFromAutoPath(
					AutoPath.UPPER_CORAL_STATION_2_TO_K,
					pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
						robot,
						pathPlannerPath,
						scoringCommand,
						AutoPath.UPPER_CORAL_STATION_2_TO_K.getTargetBranch(),
						tolerance
					)
				)
			);
			auto.setName("left");
			return auto;
		});
		autos.add(() -> {
			PathPlannerAutoWrapper auto = createAutoFromAutoPath(
				AutoPath.AUTO_LINE_4_TO_H,
				pathPlannerPath -> PathFollowingCommandsBuilder
					.commandAfterPath(robot, pathPlannerPath, scoringCommand, AutoPath.AUTO_LINE_6_TO_F.getTargetBranch(), tolerance)
			);
			auto.setName("center");
			return auto;
		});
		autos.add(() -> {
			PathPlannerAutoWrapper auto = PathPlannerAutoWrapper.chainAutos(
				createAutoFromAutoPath(
					AutoPath.AUTO_LINE_6_TO_F,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.commandAfterPath(robot, pathPlannerPath, scoringCommand, AutoPath.AUTO_LINE_6_TO_F.getTargetBranch(), tolerance)
				),
				createAutoFromAutoPath(
					AutoPath.F_TO_LOWER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.F_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
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
					pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
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
				)
			);
			auto.setName("right");
			return auto;
		});
		return autos;
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

	public static List<Supplier<PathPlannerAutoWrapper>> getAllIntakingAutos(Robot robot, Supplier<Command> intakingCommand, Pose2d tolerance) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (AutoPath autoPath : PathHelper.getAllIntakingPaths()) {
			autos.add(
				() -> createAutoFromAutoPath(
					autoPath,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.commandAfterPath(robot, pathPlannerPath, intakingCommand, autoPath.getTargetBranch(), tolerance)
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

	public static PathPlannerAutoWrapper createAutoFromAutoPath(AutoPath path, Function<PathPlannerPath, Command> pathFollowingCommand) {
		Optional<PathPlannerPath> pathOptional = path.getPath();

		return new PathPlannerAutoWrapper(
			pathOptional.map(pathFollowingCommand).orElse(Commands.none()),
			pathOptional.map(PathPlannerUtil::getPathStartingPose).orElse(path.getStartingPoint().getSecond()),
			path.getPathName(),
			pathOptional.isPresent()
		);
	}

}
