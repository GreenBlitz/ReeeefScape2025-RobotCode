package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.statemachine.superstructure.ScoreLevel;

public class RobotTasksManager {

	private final RobotCommander robotCommander;

	public RobotTasksManager(RobotCommander robotCommander) {
		this.robotCommander = robotCommander;
	}

	public Command completeAutoScore(ScoreLevel scoreLevel) {
		return new SequentialCommandGroup(
			robotCommander.preScore(),
			robotCommander.score(),
			robotCommander.exitScore(),
			robotCommander.setState(RobotState.DRIVE)
		);
	}

}
