package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.math.AngleTransform;
import frc.utils.math.ToleranceMath;

import java.util.function.Supplier;

public class PathFollowingCommandsBuilder {

	public static Command commandDuringPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier, Pose2d tolerance) {
		return new ParallelCommandGroup(commandSupplier.get(), followAdjustedPathThenStop(robot, path, tolerance));
	}

	public static Command deadlinePathWithCommand(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier) {
		return new ParallelDeadlineGroup(commandSupplier.get(), followAdjustedPath(robot, path));
	}

	public static Command commandAfterPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier, Pose2d tolerance) {
		return new SequentialCommandGroup(followAdjustedPathThenStop(robot, path, tolerance), commandSupplier.get());
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

	public static Command followPathOrPathfindAndFollowPath(Swerve swerve, PathPlannerPath path, Supplier<Pose2d> currentPose) {
		return new ConditionalCommand(
			followPath(path),
			pathfindThenFollowPath(path, AutonomousConstants.getRealTimeConstraints(swerve)),
			() -> PathPlannerUtil.isRobotInPathfindingDeadband(
				currentPose.get(),
				Field.getAllianceRelative(PathPlannerUtil.getPathStartingPose(path), true, true, AngleTransform.INVERT)
			)
		);
	}

	public static Command moveToPoseByPID(Robot robot, Pose2d targetPose) {
		return robot.getSwerve().getCommandsBuilder().moveToPoseByPID(robot.getPoseEstimator()::getEstimatedPose, targetPose);
	}

	public static Command followAdjustedPath(Robot robot, PathPlannerPath path) {
		return robot.getSwerve()
			.asSubsystemCommand(
				followPathOrPathfindAndFollowPath(robot.getSwerve(), path, () -> robot.getPoseEstimator().getEstimatedPose()).andThen(
					moveToPoseByPID(robot, Field.getAllianceRelative(PathPlannerUtil.getLastPathPose(path), true, true, AngleTransform.INVERT))
				),
				"Follow Adjusted " + path.name
			);
	}

	public static Command followAdjustedPathThenStop(Robot robot, PathPlannerPath path, Pose2d tolerance) {
		return robot.getSwerve()
			.asSubsystemCommand(
				followAdjustedPath(robot, path)
					.until(
						() -> ToleranceMath.isNear(
							Field.getAllianceRelative(PathPlannerUtil.getLastPathPose(path), true, true, AngleTransform.INVERT),
							robot.getPoseEstimator().getEstimatedPose(),
							tolerance
						)
					)
					.andThen(robot.getSwerve().getCommandsBuilder().resetTargetSpeeds()),
				"Follow Adjusted " + path.name + " Then Stop"
			);
	}

}
