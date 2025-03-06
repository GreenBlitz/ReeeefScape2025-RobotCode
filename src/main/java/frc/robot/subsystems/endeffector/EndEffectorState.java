package frc.robot.subsystems.endeffector;

public enum EndEffectorState {

	STOP(0),
	CORAL_INTAKE(0.7),
	ALGAE_INTAKE(-0.9),
	L1_OUTTAKE(-0.4),
	BRANCH_OUTTAKE(-0.8),
	CORAL_OUTTAKE(-0.1),
	DEFAULT(Double.NaN),
	ALGAE_OUTTAKE(0.7),
	NET_OUTTAKE(0.5),
	PROCESSOR_OUTTAKE(0.5);

	public static final double CORAL_KEEP_POWER = 0.02;
	public static final double ALGAE_KEEP_POWER = -0.04;

	private final double power;

	EndEffectorState(double power) {
		this.power = power;
	}

	public double getPower() {
		return power;
	}

}
