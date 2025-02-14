package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;

public class StateMachineConstants {

	public static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.5;
	public static final double OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS = 1;

	public static final double SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;
	public static final double INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;

	public static final Rotation2d ARM_REVERSED_LIMIT_BY_ELEVATOR = Rotation2d.fromDegrees(-30);
	public static final double ELEVATOR_HEIGHT_TO_LIMIT_ARM_METERS = 0.3;

	public static final double ELEVATOR_REVERSED_LIMIT_BY_ARM_METERS = 0.35;
	public static final Rotation2d ARM_POSITION_TO_LIMIT_ELEVATOR = Rotation2d.fromDegrees(-14);

	public static final double ELEVATOR_LIMIT_BY_SWERVE_METERS = 0.2;
	public static final double SWERVE_MAGNITUDE_TO_LIMIT_ELEVATOR_METERS_PER_SECOND = 2;
	public static final Rotation2d SWERVE_ROTATIONAL_SPEED_TO_LIMIT_ELEVATOR = Rotation2d.fromRadians(4);

	public static final double SWERVE_MAGNITUDE_LIMIT_BY_ELEVATOR_METERS_PER_SECOND = 1.5;
	public static final Rotation2d SWERVE_ROTATIONAL_SPEED_LIMIT_BY_ELEVATOR = Rotation2d.fromRadians(2.5);
	public static final double ELEVATOR_HEIGHT_TO_LIMIT_SWERVE_METERS = 0.4;

}
