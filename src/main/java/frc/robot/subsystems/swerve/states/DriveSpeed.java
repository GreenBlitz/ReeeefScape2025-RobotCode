package frc.robot.subsystems.swerve.states;

public record DriveSpeed(double translationSpeedFactor, double rotationSpeedFactor) {

	public static final DriveSpeed NORMAL = new DriveSpeed(1, 1);
	public static final DriveSpeed SLOW = new DriveSpeed(0.4, 0.4);

}
