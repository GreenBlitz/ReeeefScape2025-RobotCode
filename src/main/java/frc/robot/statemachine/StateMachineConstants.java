package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class StateMachineConstants {

	public static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.5;
	public static final double OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS = 1;

	public static final double SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;
	public static final double INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;


	public static final double LIMIT_ELEVATOR_MAGNITUDE_METERS_PER_SECOND = 1;
	public static final Rotation2d LIMIT_ELEVATOR_MAGNITUDE_RADIANS_PER_SECOND = Rotation2d.fromRadians(3);

	public static final ChassisSpeeds ELEVATOR_OPEN_MAX_DRIVE_SPEEDS = new ChassisSpeeds(
		LIMIT_ELEVATOR_MAGNITUDE_METERS_PER_SECOND,
		LIMIT_ELEVATOR_MAGNITUDE_METERS_PER_SECOND,
		LIMIT_ELEVATOR_MAGNITUDE_RADIANS_PER_SECOND.getRadians()
	);

}
