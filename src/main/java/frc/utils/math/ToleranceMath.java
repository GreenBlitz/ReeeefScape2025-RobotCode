package frc.utils.math;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.MathConstants;

public class ToleranceMath {

	public static boolean isNear(Pose2d wantedPose, Pose2d pose, Pose2d tolerance) {
		return isNear(wantedPose.getTranslation(), pose.getTranslation(), tolerance.getTranslation())
			&& isNearWrapped(wantedPose.getRotation(), pose.getRotation(), tolerance.getRotation());
	}

	public static boolean isNear(Translation2d wantedTranslation, Translation2d translation, double toleranceMeters) {
		return translation.getDistance(wantedTranslation) <= toleranceMeters;
	}

	public static boolean isNear(Translation2d wantedTranslation, Translation2d translation, Translation2d tolerance) {
		return MathUtil.isNear(wantedTranslation.getX(), translation.getX(), tolerance.getX())
			&& MathUtil.isNear(wantedTranslation.getY(), translation.getY(), tolerance.getY());
	}

	public static boolean isNearWrapped(Rotation2d wantedAngle, Rotation2d angle, Rotation2d tolerance) {
		return MathUtil.isNear(
			wantedAngle.getRadians(),
			angle.getRadians(),
			tolerance.getRadians(),
			MathConstants.HALF_CIRCLE.unaryMinus().getRadians(),
			MathConstants.HALF_CIRCLE.getRadians()
		);
	}

	public static boolean isInRange(double value, double min, double max, double tolerance) {
		return (min - tolerance) <= value && value <= (max + tolerance);
	}

	public static boolean isInRange(double value, double min, double max) {
		return isInRange(value, min, max, 0);
	}

	public static double applyDeadband(double value, double deadband) {
		return Math.abs(value) <= deadband ? 0 : value;
	}

	public static Rotation2d clamp(Rotation2d angle, Rotation2d maxAngle) {
		return Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), -maxAngle.getRadians(), maxAngle.getRadians()));
	}

	public static Rotation2d clamp(Rotation2d angle, Rotation2d minAngle, Rotation2d maxAngle) {
		return Rotation2d.fromRadians(MathUtil.clamp(angle.getRadians(), minAngle.getRadians(), maxAngle.getRadians()));
	}

}
