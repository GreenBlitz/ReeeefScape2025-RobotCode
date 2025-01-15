package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

import java.util.function.Supplier;

public class SequencesBuilder {

	public static Command intake(Robot robot, PathPlannerPath path, Supplier<Command> intakeCommand) {
		return new ParallelRaceGroup(RobotAutoHelper.followPathOrDriveToPathEnd(robot, path), intakeCommand.get());
	}

	public static Command intakeShoot(Robot robot, PathPlannerPath path, Supplier<Command> intakeCommand, Supplier<Command> shootingCommand) {
		return new SequentialCommandGroup(intake(robot, path, intakeCommand), shootingCommand.get());
	}

	public static Command shooting(Robot robot, PathPlannerPath path, Supplier<Command> preShootingCommand, Supplier<Command> shootingCommand) {
		return new SequentialCommandGroup(
			new ParallelDeadlineGroup(RobotAutoHelper.followPathOrDriveToPathEnd(robot, path), preShootingCommand.get()),
			shootingCommand.get()
		);
	}

	public static Command commandDuringPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier) {
		return new ParallelCommandGroup(RobotAutoHelper.followPathOrDriveToPathEnd(robot, path), commandSupplier.get());
	}

	public static Command commandAfterPath(Robot robot, PathPlannerPath path, Supplier<Command> commandSupplier) {
		return new SequentialCommandGroup(RobotAutoHelper.followPathOrDriveToPathEnd(robot, path), commandSupplier.get());
	}

	public static Command feedAndScore(
		Robot robot,
		PathPlannerPath pathToSource,
		PathPlannerPath pathFromSource,
		Supplier<Command> feedingCommand,
		Supplier<Command> scoringCommand
	) {
		return new SequentialCommandGroup(
			commandAfterPath(robot, pathToSource, feedingCommand),
			commandAfterPath(robot, pathFromSource, scoringCommand)
		);
	}

}
