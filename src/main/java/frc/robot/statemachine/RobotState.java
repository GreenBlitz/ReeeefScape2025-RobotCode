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
		DRIVE.neighbors = new RobotState[] {ALGAE_INTAKE, TRANSFER_ALGAE_TO_END_EFFECTOR};
		INTAKE_WITH_AIM_ASSIST.neighbors = new RobotState[] {};
		INTAKE_WITHOUT_AIM_ASSIST.neighbors = new RobotState[] {};
		CORAL_OUTTAKE.neighbors = new RobotState[] {};
		ALIGN_REEF.neighbors = new RobotState[] {};
		ARM_PRE_SCORE.neighbors = new RobotState[] {};
		PRE_SCORE.neighbors = new RobotState[] {};
		SCORE_WITHOUT_RELEASE.neighbors = new RobotState[] {};
		SCORE.neighbors = new RobotState[] {};
		ALGAE_REMOVE.neighbors = new RobotState[] {};
		STAY_IN_PLACE.neighbors = new RobotState[] {};
		ALGAE_OUTTAKE_FROM_END_EFFECTOR.neighbors = new RobotState[] {};
		ALGAE_OUTTAKE_FROM_INTAKE.neighbors = new RobotState[] {};
		ALGAE_INTAKE.neighbors = new RobotState[] {DRIVE};
		TRANSFER_ALGAE_TO_END_EFFECTOR.neighbors = new RobotState[] {};
		HOLD_ALGAE.neighbors = new RobotState[] {};
		AUTO_PRE_NET.neighbors = new RobotState[] {};
		PRE_NET.neighbors = new RobotState[] {};
		NET.neighbors = new RobotState[] {};
		PROCESSOR_SCORE.neighbors = new RobotState[] {};
		SOFT_CLOSE.neighbors = new RobotState[] {DRIVE};
		PRE_CLIMB_WITH_AIM_ASSIST.neighbors = new RobotState[] {};
		PRE_CLIMB_WITHOUT_AIM_ASSIST.neighbors = new RobotState[] {};
		CLIMB_WITHOUT_LIMIT_SWITCH.neighbors = new RobotState[] {};
		CLIMB_WITH_LIMIT_SWITCH.neighbors = new RobotState[] {};
		MANUAL_CLIMB.neighbors = new RobotState[] {};
		EXIT_CLIMB.neighbors = new RobotState[] {};
		STOP_CLIMB.neighbors = new RobotState[] {};
		CLOSE_CLIMB.neighbors = new RobotState[] {};
	}

	public RobotState[] getNeighbors() {
		return neighbors;
	}

}
