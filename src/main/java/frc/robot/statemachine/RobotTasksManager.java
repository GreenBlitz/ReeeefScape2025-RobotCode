package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.scoringhelpers.ScoringHelpers;

public class RobotTasksManager {

	private final RobotCommander robotCommander;

	public RobotTasksManager(RobotCommander robotCommander) {
		this.robotCommander = robotCommander;
	}

	public Command completeAutoScore() {
		return new SequentialCommandGroup(robotCommander.preScore(), robotCommander.score(), robotCommander.exitScore(), robotCommander.setState(RobotState.DRIVE));
	}

	public Command scoreForButton() {
		return new SequentialCommandGroup(robotCommander.scoreWithoutRelease().until(robotCommander::isReadyToScore), robotCommander.score());
	}

	public Command fullyPreScore() {
		return new SequentialCommandGroup(
			robotCommander.armPreScore().until(robotCommander::isAtOpenSuperstructureDistanceFromReef),
			robotCommander.preScore()
				.until(() -> robotCommander.isPreScoreReady(ScoringHelpers.targetScoreLevel, ScoringHelpers.getTargetBranch())),
			robotCommander.scoreWithoutRelease()
		);
	}

}
