package frc.robot.subsystems.elevator;

public enum ElevatorState {

	CLOSED(0.02),
	INTAKE(0.02),
	OUTTAKE(0.02),
	PRE_L1(0),
	L1(0.02),
	PRE_L2(0.02),
	L2(0.02),
	PRE_L3(0.16),
	L3(0.16),
	WHILE_DRIVE_L4(0.35),
	PRE_L4(1.18),
	L4(1.18),
	POST_LOW_ALGAE_REMOVE(0.02),
	LOW_ALGAE_REMOVE(0.02),
	POST_HIGH_ALGAE_REMOVE(0.08),
	HIGH_ALGAE_REMOVE(0.08),
	HIGH_ALGAE_REMOVE_FROM_L4(0.8),
	POST_HIGH_ALGAE_REMOVE_FROM_L4(0.85),
	LOW_ALGAE_REMOVE_FROM_L4(0.5),
	POST_LOW_ALGAE_REMOVE_FROM_L4(0.55),
	WHILE_DRIVE_NET(0.35),
	PRE_NET(1.18),
	NET(1.18);


	private final double heightMeters;

	ElevatorState(double heightMeters) {
		this.heightMeters = heightMeters;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

}
