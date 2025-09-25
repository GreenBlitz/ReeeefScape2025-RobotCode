package frc.robot.statemachine;

public enum RobotState {

	STAY_IN_PLACE(new RobotState[] {}),
	DRIVE,
	INTAKE_WITH_AIM_ASSIST(new RobotState[] {}),
	INTAKE_WITHOUT_AIM_ASSIST(new RobotState[] {}),
	CORAL_OUTTAKE(new RobotState[] {}),
	ALIGN_REEF(new RobotState[] {}),
	ARM_PRE_SCORE(new RobotState[] {}),
	PRE_SCORE(new RobotState[] {}),
	SCORE_WITHOUT_RELEASE(new RobotState[] {}),
	SCORE(new RobotState[] {}),
	ALGAE_REMOVE(new RobotState[] {}),
	ALGAE_OUTTAKE_FROM_END_EFFECTOR(new RobotState[] {}),
	ALGAE_OUTTAKE_FROM_INTAKE(new RobotState[] {}),
	ALGAE_INTAKE,
	TRANSFER_ALGAE_TO_END_EFFECTOR,
	HOLD_ALGAE,
	AUTO_PRE_NET(new RobotState[] {}),
	PRE_NET,
	NET,
	PROCESSOR_SCORE(new RobotState[] {}),
	PRE_CLIMB_WITH_AIM_ASSIST(new RobotState[] {}),
	PRE_CLIMB_WITHOUT_AIM_ASSIST(new RobotState[] {}),
	CLIMB_WITHOUT_LIMIT_SWITCH(new RobotState[] {}),
	CLIMB_WITH_LIMIT_SWITCH(new RobotState[] {}),
	MANUAL_CLIMB(new RobotState[] {}),
	EXIT_CLIMB(new RobotState[] {}),
	STOP_CLIMB(new RobotState[] {}),
	CLOSE_CLIMB(new RobotState[] {});

	private RobotState[] neighbors;

	RobotState(RobotState[] neighbors) {
		this.neighbors = neighbors;
	}

	public RobotState[] getNeighbors() {
		return neighbors;
	}

}
