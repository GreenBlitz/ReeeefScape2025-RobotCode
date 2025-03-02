package frc.utils.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;


public class PoseMath {

	public static Translation2d getRelativeTranslation(Translation2d relativeTo, Translation2d toRelative) {
		return toRelative.minus(relativeTo);
	}

	public static Translation2d getRelativeTranslation(Pose2d relativeTo, Translation2d toRelative) {
		return getRelativeTranslation(relativeTo.getTranslation(), toRelative).rotateBy(relativeTo.getRotation().unaryMinus());
	}

	public static double calculateTranslationDistanceFromLine(Translation2d translation, double lineSlope, double lineIntercept) {
		// Convert the line equation to standard form Ax + By + C = 0
		double A = -lineSlope;
		double B = 1;
		double C = -lineIntercept;

		// Distance formula: |Ax0 + By0 + C| / sqrt(A^2 + B^2)
		return Math.abs(A * translation.getX() + B * translation.getY() + C) / Math.sqrt(A * A + B * B);
	}

}
