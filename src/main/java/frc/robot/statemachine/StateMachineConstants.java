package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class StateMachineConstants {

	public static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.5;
	public static final double OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS = 1;
	public static final double SAFE_ZONE_DISTANCE_FROM_SCORING_POSITION = 1;

	public static final Pose2d SAFE_ZONE_DISTANCES_FROM_CENTER_OF_SAFE_ZONE = new Pose2d(1, 2, new Rotation2d());

	public static final double SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;
	public static final double INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;

}
