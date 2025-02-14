package frc.robot.subsystems.swerve.states;

public record DriveSpeed(double magnitudeSpeedFactor, double rotationSpeedFactor) {

	public static final DriveSpeed NORMAL = new DriveSpeed(1, 1);
	public static final DriveSpeed SLOW = new DriveSpeed(0.4, 0.4);

	public DriveSpeed combine(DriveSpeed other) {
		double magnitudeSpeedFactor = Math.min(this.magnitudeSpeedFactor, other.magnitudeSpeedFactor);
		double rotationalSpeedFactor = Math.min(this.rotationSpeedFactor, other.rotationSpeedFactor);
		return new DriveSpeed(magnitudeSpeedFactor, rotationalSpeedFactor);
	}

}
