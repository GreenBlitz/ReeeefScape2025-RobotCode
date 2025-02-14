package frc.robot.subsystems.swerve.states;

public record DriveSpeedLimit(double magnitudeSpeedFactor, double rotationSpeedFactor) {

	public static final DriveSpeedLimit NORMAL = new DriveSpeedLimit(1, 1);
	public static final DriveSpeedLimit SLOW = new DriveSpeedLimit(0.4, 0.4);

	public DriveSpeedLimit combine(DriveSpeedLimit other) {
		double magnitudeSpeedFactor = Math.min(this.magnitudeSpeedFactor, other.magnitudeSpeedFactor);
		double rotationalSpeedFactor = Math.min(this.rotationSpeedFactor, other.rotationSpeedFactor);
		return new DriveSpeedLimit(magnitudeSpeedFactor, rotationalSpeedFactor);
	}

}
