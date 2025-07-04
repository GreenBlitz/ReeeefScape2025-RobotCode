package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;

public class Tolerances {

	public static final double ELEVATOR_HEIGHT_METERS = 0.05;

	public static final Rotation2d ARM_INTERPOLATION_POSITION = Rotation2d.fromDegrees(0.5);
	public static final Rotation2d ARM_POSITION = Rotation2d.fromDegrees(1.5);
	public static final Rotation2d ALGAE_RELEASE_ARM_POSITION = Rotation2d.fromDegrees(10);

	public static final Pose2d REEF_RELATIVE_SCORING_POSITION = new Pose2d(0.15, 0.025, Rotation2d.fromDegrees(3.5));
	public static final Pose2d REEF_RELATIVE_SCORING_DEADBANDS = new Pose2d(1, 1, Rotation2d.fromRadians(2));
	public static final Pose2d NET_OPENING_SUPERSTRUCTURE_POSITION_METERS = new Pose2d(0.07, 0.07, Rotation2d.fromDegrees(2));
	public static final Pose2d NET_SCORING_POSITION_METERS = new Pose2d(0.07, 0.07, Rotation2d.fromDegrees(10));


	public static final Pose2d REEF_RELATIVE_OPEN_SUPERSTRUCTURE_POSITION = new Pose2d(
		StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS,
		1,
		Rotation2d.fromDegrees(15)
	);
	public static final Pose2d REEF_RELATIVE_OPEN_SUPERSTRUCTURE_DEADBANDS = new Pose2d(2, 2, Rotation2d.fromRadians(4));

	public static final Pose2d REEF_RELATIVE_L1_SCORING_POSITION = new Pose2d(
		0.2,
		Field.REEF_SIDE_LENGTH_METERS / 2.0,
		Rotation2d.fromDegrees(3)
	);
	public static final Pose2d REEF_RELATIVE_L1_SCORING_DEADBANDS = new Pose2d(0.2, 1, Rotation2d.fromRadians(0.1));

	public static final Pose2d REEF_RELATIVE_L1_OPEN_SUPERSTRUCTURE_POSITION = new Pose2d(
		StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS,
		Field.REEF_SIDE_LENGTH_METERS / 2.0,
		Rotation2d.fromDegrees(3)
	);
	public static final Pose2d REEF_RELATIVE_L1_OPEN_SUPERSTRUCTURE_DEADBANDS = new Pose2d(4, 1, Rotation2d.fromRadians(0.1));

	public static final Pose2d PROCESSOR_RELATIVE_SCORING_POSITION = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(6));
	public static final Pose2d PROCESSOR_RELATIVE_SCORING_DEADBANDS = new Pose2d(3.8, 0.5, Rotation2d.fromRadians(2));

	public static final Rotation2d PIVOT = Rotation2d.fromDegrees(3);

}
