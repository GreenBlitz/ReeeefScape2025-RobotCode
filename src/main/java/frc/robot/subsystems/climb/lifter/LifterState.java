package frc.robot.subsystems.climb.lifter;

import edu.wpi.first.math.geometry.Rotation2d;

public enum LifterState {

	HOLD(Rotation2d.fromDegrees(0), 0),
	FORWARD(Rotation2d.fromDegrees(0), 0.2),
	BACKWARD(Rotation2d.fromDegrees(0), -0.2),
	CLIMB_WITHOUT_LIMIT_SWITCH(Rotation2d.fromDegrees(15), -0.7),
	CLIMB_WITH_LIMIT_SWITCH(Rotation2d.fromDegrees(15), -0.7),
	MANUAL_CLIMB(null, -0.5),
	DEPLOY(Rotation2d.fromDegrees(60), 1),
	CLOSE(Rotation2d.fromDegrees(-6.5), -0.8);

	private final Rotation2d targetPosition;
	private final double power;

	LifterState(Rotation2d targetPosition, double power) {
		this.targetPosition = targetPosition;
		this.power = power;
	}

	public Rotation2d getTargetPosition() {
		return targetPosition;
	}

	public double getPower() {
		return power;
	}

}
