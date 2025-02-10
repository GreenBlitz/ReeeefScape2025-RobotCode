package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.math.AngleTransform;
import frc.utils.math.ToleranceMath;

import java.util.function.Supplier;

public class PathFollowingCommandsBuilder {

	public static Command commandDuringPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier, Pose2d tolerance) {
		return new ParallelCommandGroup(commandSupplier.get(), followAdjustedPath(robot, path, tolerance));
	}

	public static Command commandAfterPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier, Pose2d tolerance) {
		return new SequentialCommandGroup(followAdjustedPath(robot, path, tolerance), commandSupplier.get());
	}


	public static Command followPath(PathPlannerPath path) {
		return AutoBuilder.followPath(path);
	}

	public static Command pathfindToPose(Pose2d targetPose, PathConstraints pathfindingConstraints) {
		return AutoBuilder.pathfindToPose(targetPose, pathfindingConstraints);
	}

	public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints pathfindingConstraints) {
		return AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints);
	}

	public static Command pathfindThenFollowPath(
		PathPlannerPath path,
		PathConstraints pathfindingConstraints,
		double velocityBetweenPathfindingToPathFollowingMetersPerSecond
	) {
		return AutoBuilder
			.pathfindToPose(
				Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path), true, true, AngleTransform.INVERT),
				pathfindingConstraints,
				velocityBetweenPathfindingToPathFollowingMetersPerSecond
			)
			.andThen(followPath(path));
	}

	public static Command followPathOrPathfindAndFollowPath(Robot robot, PathPlannerPath path) {
		return new ConditionalCommand(
			followPath(path),
			pathfindThenFollowPath(path, AutonomousConstants.REAL_TIME_CONSTRAINTS),
			() -> PathPlannerUtil.isRobotInPathfindingDeadband(
				robot.getPoseEstimator().getEstimatedPose(),
				Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path), true, true, AngleTransform.INVERT)
			)
		);
	}

	public static Command moveToPoseByPID(Robot robot, Pose2d targetPose) {
		return robot.getSwerve().getCommandsBuilder().moveToPoseByPID(robot.getPoseEstimator()::getEstimatedPose, targetPose);
	}

	public static Command followAdjustedPath(Robot robot, PathPlannerPath path, Pose2d tolerance) {
		return followPathOrPathfindAndFollowPath(robot, path)
			.andThen(moveToPoseByPID(robot, Field.getAllianceRelative(PathPlannerUtil.getLastPathPose(path), true, true, AngleTransform.INVERT)))
			.until(
				() -> ToleranceMath.isNear(
					Field.getAllianceRelative(PathPlannerUtil.getLastPathPose(path), true, true, AngleTransform.INVERT),
					robot.getPoseEstimator().getEstimatedPose(),
					tolerance
				)
			)
			.andThen(robot.getSwerve().getCommandsBuilder().resetTargetSpeeds());
	}

}
