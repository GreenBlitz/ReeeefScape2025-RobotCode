package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Tolerances {

	public static final double ELEVATOR_HEIGHT_METERS = 0.05;

	public static final Rotation2d ARM_POSITION = Rotation2d.fromDegrees(3);
	public static final Rotation2d ALGAE_RELEASE_ARM_POSITION = Rotation2d.fromDegrees(10);

	public static final Pose2d REEF_RELATIVE_SCORING_POSITION = new Pose2d(0.035, 0.035, Rotation2d.fromDegrees(2));
	public static final Pose2d REEF_RELATIVE_SCORING_DEADBANDS = new Pose2d(0.5, 0.5, Rotation2d.fromRadians(2));
	public static final Rotation2d HEADING_FOR_NET = Rotation2d.fromDegrees(10);
	public static final Rotation2d HEADING_FOR_NET_DEADBAND = Rotation2d.fromDegrees(1);

	public static final Pose2d REEF_RELATIVE_OPEN_SUPERSTRUCTURE_POSITION = new Pose2d(
		StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS,
		0.25,
		Rotation2d.fromDegrees(10)
	);
	public static final Pose2d REEF_RELATIVE_OPEN_SUPERSTRUCTURE_DEADBANDS = new Pose2d(2, 2, Rotation2d.fromDegrees(4));

	public static final Pose2d PROCESSOR_RELATIVE_SCORING_POSITION = new Pose2d(0.2, 0.2, Rotation2d.fromDegrees(3));
	public static final Pose2d PROCESSOR_RELATIVE_SCORING_DEADBANDS = new Pose2d(0.5, 0.5, Rotation2d.fromRadians(2));

}
