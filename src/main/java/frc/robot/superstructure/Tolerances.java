package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Tolerances {

	public static final double ELEVATOR_HEIGHT_METERS = 0.01;

	public static final Rotation2d ARM_POSITION = Rotation2d.fromDegrees(1.5);

	public static final Pose2d SWERVE_REEF_POSE = new Pose2d(0.2, 0.2, new Rotation2d(0.2));

}
