package frc.robot.statemachine;


public enum RobotState {

	STAY_IN_PLACE,
	DRIVE,
	INTAKE_WITH_AIM_ASSIST,
	INTAKE_WITHOUT_AIM_ASSIST,
	CORAL_OUTTAKE,
	ALIGN_REEF,
	ARM_PRE_SCORE,
	PRE_SCORE,
	SCORE_WITHOUT_RELEASE,
	SCORE,
	ALGAE_REMOVE,
	ALGAE_OUTTAKE_FROM_END_EFFECTOR,
	ALGAE_OUTTAKE_FROM_INTAKE,
	ALGAE_INTAKE,
	TRANSFER_ALGAE_TO_END_EFFECTOR,
	HOLD_ALGAE,
	AUTO_PRE_NET,
	PRE_NET,
	NET,
	PROCESSOR_SCORE,
	SOFT_CLOSE,
	PRE_CLIMB_WITH_AIM_ASSIST,
	PRE_CLIMB_WITHOUT_AIM_ASSIST,
	CLIMB_WITHOUT_LIMIT_SWITCH,
	CLIMB_WITH_LIMIT_SWITCH,
	MANUAL_CLIMB,
	EXIT_CLIMB,
	STOP_CLIMB,
	CLOSE_CLIMB;

	private RobotState[] neighbors;

	static {
		DRIVE.neighbors = new RobotState[] {
			ALGAE_INTAKE,
			TRANSFER_ALGAE_TO_END_EFFECTOR,
			ALGAE_OUTTAKE_FROM_INTAKE,
			STAY_IN_PLACE,
			INTAKE_WITH_AIM_ASSIST,
			INTAKE_WITHOUT_AIM_ASSIST,
			CORAL_OUTTAKE,
			ALIGN_REEF,
			AUTO_PRE_NET,
			ARM_PRE_SCORE,
			PRE_SCORE,
			SCORE_WITHOUT_RELEASE,
			SCORE,
			ALGAE_REMOVE,
			ALGAE_OUTTAKE_FROM_END_EFFECTOR,
			PROCESSOR_SCORE,
			SOFT_CLOSE,
			PRE_CLIMB_WITH_AIM_ASSIST,
			PRE_CLIMB_WITHOUT_AIM_ASSIST,
			CLIMB_WITHOUT_LIMIT_SWITCH,
			CLIMB_WITH_LIMIT_SWITCH,
			MANUAL_CLIMB,
			EXIT_CLIMB,
			STOP_CLIMB,
			CLOSE_CLIMB};
		INTAKE_WITH_AIM_ASSIST.neighbors = new RobotState[] {DRIVE};
		INTAKE_WITHOUT_AIM_ASSIST.neighbors = new RobotState[] {DRIVE};
		CORAL_OUTTAKE.neighbors = new RobotState[] {DRIVE};
		ALIGN_REEF.neighbors = new RobotState[] {DRIVE};
		ARM_PRE_SCORE.neighbors = new RobotState[] {DRIVE};
		PRE_SCORE.neighbors = new RobotState[] {DRIVE};
		SCORE_WITHOUT_RELEASE.neighbors = new RobotState[] {DRIVE};
		SCORE.neighbors = new RobotState[] {DRIVE};
		ALGAE_REMOVE.neighbors = new RobotState[] {DRIVE};
		STAY_IN_PLACE.neighbors = new RobotState[] {DRIVE};
		ALGAE_OUTTAKE_FROM_END_EFFECTOR.neighbors = new RobotState[] {DRIVE};
		ALGAE_OUTTAKE_FROM_INTAKE.neighbors = new RobotState[] {DRIVE};
		ALGAE_INTAKE.neighbors = new RobotState[] {DRIVE};
		TRANSFER_ALGAE_TO_END_EFFECTOR.neighbors = new RobotState[] {HOLD_ALGAE};
		HOLD_ALGAE.neighbors = new RobotState[] {ALGAE_OUTTAKE_FROM_END_EFFECTOR, PRE_NET};
		AUTO_PRE_NET.neighbors = new RobotState[] {DRIVE};
		PRE_NET.neighbors = new RobotState[] {NET, HOLD_ALGAE};
		NET.neighbors = new RobotState[] {SOFT_CLOSE};
		PROCESSOR_SCORE.neighbors = new RobotState[] {DRIVE};
		SOFT_CLOSE.neighbors = new RobotState[] {DRIVE};
		PRE_CLIMB_WITH_AIM_ASSIST.neighbors = new RobotState[] {DRIVE};
		PRE_CLIMB_WITHOUT_AIM_ASSIST.neighbors = new RobotState[] {DRIVE};
		CLIMB_WITHOUT_LIMIT_SWITCH.neighbors = new RobotState[] {DRIVE};
		CLIMB_WITH_LIMIT_SWITCH.neighbors = new RobotState[] {DRIVE};
		MANUAL_CLIMB.neighbors = new RobotState[] {DRIVE};
		EXIT_CLIMB.neighbors = new RobotState[] {DRIVE};
		STOP_CLIMB.neighbors = new RobotState[] {DRIVE};
		CLOSE_CLIMB.neighbors = new RobotState[] {DRIVE};
	}

	public RobotState[] getNeighbors() {
		return neighbors;
	}

}
