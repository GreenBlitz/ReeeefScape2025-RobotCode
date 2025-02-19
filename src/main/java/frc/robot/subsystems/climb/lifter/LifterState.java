package frc.robot.subsystems.climb.lifter;

public enum LifterState {

	HOLD(0, 0),
	FORWARD(0, 0.2),
	BACKWARD(0, -0.2),
	CLIMB(0.05, -0.9),
	DEPLOY(0.4, 0.9);

	private final double targetPositionMeters;
	private final double power;

	LifterState(double targetPositionMeters, double power) {
		this.targetPositionMeters = targetPositionMeters;
		this.power = power;
	}

	public double getTargetPositionMeters() {
		return targetPositionMeters;
	}

	public double getPower() {
		return power;
	}

}
