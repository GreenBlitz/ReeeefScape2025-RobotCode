package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public class Tolerances {

	public static final double ELEVATOR_HEIGHT_METERS = 0.05;

	public static final Rotation2d ARM_POSITION = Rotation2d.fromDegrees(3);
	
	public static final double CHASSIS_POSITION_METERS =  0.02;
	public static final Rotation2d CHASSIS_ROTATION = Rotation2d.fromDegrees(3);
	public static final Rotation2d VELOCITY_DEADBAND_ANGLES_PER_SECOND = Rotation2d.fromRotations(1);

}
