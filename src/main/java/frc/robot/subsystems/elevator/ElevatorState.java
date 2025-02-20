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
	PRE_LOW_ALGAE_REMOVE(0.02),
	LOW_ALGAE_REMOVE(0.02),
	PRE_HIGH_ALGAE_REMOVE(0.02),
	HIGH_ALGAE_REMOVE(0.02);


	private final double heightMeters;

	ElevatorState(double heightMeters) {
		this.heightMeters = heightMeters;
	}

	public double getHeightMeters() {
		return heightMeters;
	}

}
