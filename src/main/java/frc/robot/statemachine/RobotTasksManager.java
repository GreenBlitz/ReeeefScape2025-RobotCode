package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotTasksManager {

	private final RobotCommander robotCommander;

	public RobotTasksManager(RobotCommander robotCommander) {
		this.robotCommander = robotCommander;
	}

	public Command completeAutoScore() {
		return new SequentialCommandGroup(
			robotCommander.preScore(),
			robotCommander.score(),
			robotCommander.exitScore(),
			robotCommander.setState(RobotState.DRIVE)
		);
	}

}
