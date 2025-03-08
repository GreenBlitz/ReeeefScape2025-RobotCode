package frc.robot.autonomous;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static final Pose2d TARGET_POSE_TOLERANCES = new Pose2d(0.035, 0.035, Rotation2d.fromDegrees(2));

	public static final double DEFAULT_AUTO_DRIVE_POWER = -0.5;

	public static final double DEFAULT_AUTO_DRIVE_TIME_SECONDS = 0.6;

	public static final double INTAKING_TIMEOUT_SECONDS = 6;

	public static PathConstraints getRealTimeConstraints(Swerve swerve) {
		return new PathConstraints(
			swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
			2.5,
			swerve.getConstants().maxRotationalVelocityPerSecond().getRadians(),
			RealSwerveConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND
		);
	}

	public static PathConstraints getAutoTimeConstraints(Swerve swerve) {
		return new PathConstraints(
			swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
			5.5,
			swerve.getConstants().maxRotationalVelocityPerSecond().getRadians(),
			RealSwerveConstants.MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND
		);
	}

	public static class LinkedWaypoints {

		public static final Pair<String, Pose2d> AUTO_LINE_1 = Pair.of("AL1", new Pose2d(7.58, 7.26, Rotation2d.fromDegrees(-137)));

		public static final Pair<String, Pose2d> AUTO_LINE_2 = Pair.of("AL2", new Pose2d(7.58, 6.17, Rotation2d.fromDegrees(-145)));

		public static final Pair<String, Pose2d> AUTO_LINE_3 = Pair.of("AL3", new Pose2d(7.58, 5.07, Rotation2d.fromDegrees(-157)));

		public static final Pair<String, Pose2d> AUTO_LINE_4 = Pair.of("AL4", new Pose2d(7.58, 4.02, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> AUTO_LINE_5 = Pair.of("AL5", new Pose2d(7.58, 3, Rotation2d.fromDegrees(157)));

		public static final Pair<String, Pose2d> AUTO_LINE_6 = Pair.of("AL6", new Pose2d(7.58, 1.9, Rotation2d.fromDegrees(152)));

		public static final Pair<String, Pose2d> AUTO_LINE_7 = Pair.of("AL7", new Pose2d(7.58, 0.81, Rotation2d.fromDegrees(137)));

		public static final Pair<String, Pose2d> UPPER_CORAL_STATION_2 = Pair.of("US2", new Pose2d(1.63, 7.34, Rotation2d.fromDegrees(-54)));

		public static final Pair<String, Pose2d> LOWER_CORAL_STATION_2 = Pair.of("LS2", new Pose2d(1.61, 0.7, Rotation2d.fromDegrees(54)));

		public static final Pair<String, Pose2d> A = Pair.of("A", new Pose2d(3.16, 4.19, Rotation2d.fromDegrees(0)));

		public static final Pair<String, Pose2d> B = Pair.of("B", new Pose2d(3.16, 3.86, Rotation2d.fromDegrees(0)));

		public static final Pair<String, Pose2d> C = Pair.of("C", new Pose2d(3.65, 2.96, Rotation2d.fromDegrees(60)));

		public static final Pair<String, Pose2d> D = Pair.of("D", new Pose2d(3.97, 2.8, Rotation2d.fromDegrees(60)));

		public static final Pair<String, Pose2d> E = Pair.of("E", new Pose2d(5.01, 2.79, Rotation2d.fromDegrees(120)));

		public static final Pair<String, Pose2d> F = Pair.of("F", new Pose2d(5.3, 2.94, Rotation2d.fromDegrees(120)));

		public static final Pair<String, Pose2d> G = Pair.of("G", new Pose2d(5.83, 3.86, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> H = Pair.of("H", new Pose2d(5.83, 4.19, Rotation2d.fromDegrees(180)));

		public static final Pair<String, Pose2d> I = Pair.of("I", new Pose2d(5.3, 5.08, Rotation2d.fromDegrees(-120)));

		public static final Pair<String, Pose2d> J = Pair.of("J", new Pose2d(5.02, 5.25, Rotation2d.fromDegrees(-120)));

		public static final Pair<String, Pose2d> K = Pair.of("K", new Pose2d(3.95, 5.26, Rotation2d.fromDegrees(-60)));

		public static final Pair<String, Pose2d> L = Pair.of("L", new Pose2d(3.68, 5.1, Rotation2d.fromDegrees(-60)));

	}

}
