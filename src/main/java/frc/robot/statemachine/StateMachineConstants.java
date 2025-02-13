package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.arm.ArmConstants;

public class StateMachineConstants {

	public static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.5;
	public static final double OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS = 1;

	public static final double SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;
	public static final double INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS = 0.5;

	public static final Rotation2d ELEVATOR_CLOSED_REVERSED_SOFTWARE_LIMIT = Rotation2d.fromDegrees(-13);
	public static final Rotation2d ELEVATOR_OPEN_REVERSED_SOFTWARE_LIMIT = Rotation2d.fromDegrees(-30);
	public static final double ELEVATOR_HEIGHT_METERS_TO_CHANGE_SOFT_LIMIT = 0.3;

	public static final double LIMIT_ELEVATOR_MAGNITUDE_METERS_PER_SECOND = 2;
	public static final Rotation2d LIMIT_ELEVATOR_MAGNITUDE_RADIANS_PER_SECOND = Rotation2d.fromRadians(4);

	public static final double ARM_OPEN_REVERSED_SOFTWARE_LIMIT = ArmConstants.ELEVATOR_HEIGHT_METERS_TO_CHANGE_SOFT_LIMIT;
	public static final Rotation2d ARM_POSITION_TO_CHANGE_SOFT_LIMIT = ArmConstants.ELEVATOR_CLOSED_REVERSED_SOFTWARE_LIMIT;

	public static final double LIMIT_FOR_SPEEDS_METERS = 0.55;
	public static final double LIMIT_BY_SPEEDS = 0.3;

}
