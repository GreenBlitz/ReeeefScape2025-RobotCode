package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.ReefSide;

public class Tolerances {

	public static final double ELEVATOR_HEIGHT_METERS = 0.01;

	public static final Rotation2d ARM_POSITION = Rotation2d.fromDegrees(1.5);

	public static final Pose2d BRANCH_SCORE_POSE = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(3));
	public static final Pose2d BRANCH_SCORE_VELOCITY_DEADBANDS = new Pose2d(0.1, 0.1, Rotation2d.fromRadians(0.1));

	public static final Pose2d L1_SCORE_POSE = new Pose2d(Field.REEF_SIDE_LENGTH_METERS / 2.0, 0.05, Rotation2d.fromDegrees(3));
	public static final Pose2d L1_SCORE_VELOCITY_DEADBANDS = new Pose2d(1, 0.1, Rotation2d.fromRadians(0.1));

}
