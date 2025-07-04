package frc.robot.subsystems.algaeIntake.rollers;

public enum RollersState {

	IDLE(0.01),
	INTAKE(0.5),
	TRANSFER_TO_END_EFFECTOR(-0.4),
	OUTTAKE(-0.6),
	SLOW_OUTTAKE(-0.4),
	CLIMB(-0.2);

	private final double power;

	RollersState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
