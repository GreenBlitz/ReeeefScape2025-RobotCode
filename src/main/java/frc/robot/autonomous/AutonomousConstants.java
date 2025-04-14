package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static final Pose2d TARGET_POSE_TOLERANCES = new Pose2d(0.035, 0.035, Rotation2d.fromDegrees(2));

	public static final double DEFAULT_AUTO_DRIVE_POWER = -0.3;

	public static final double DEFAULT_AUTO_DRIVE_TIME_SECONDS = 1;

	public static final double INTAKING_TIMEOUT_SECONDS = 4;

	public static PathConstraints getRealTimeConstraints(Swerve swerve) {
		return new PathConstraints(
			swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
			2.5, // RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
			swerve.getConstants().maxRotationalVelocityPerSecond().getRadians(),
			RealSwerveConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND
		);
	}

	public static PathConstraints getRealTimeConstraintsForAuto(Swerve swerve) {
		return new PathConstraints(
			swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
			3.5, // RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
			swerve.getConstants().maxRotationalVelocityPerSecond().getRadians(),
			RealSwerveConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND
		);
	}

}
