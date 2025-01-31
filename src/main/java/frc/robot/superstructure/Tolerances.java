package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Tolerances {

	public static final double ELEVATOR_HEIGHT_METERS = 0.01;

	public static final Rotation2d ARM_POSITION = Rotation2d.fromDegrees(1.5);

	public static final Pose2d SWERVE_BRANCH_SCORE = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(3));

	public static final Rotation2d SWERVE_ROTATIONAL_VELOCITY_PER_SECOND_DEADBAND = Rotation2d.fromRadians(0.05);


}
