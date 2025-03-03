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

	/**
	 * A function to calculate the slope and intercept
	 *
	 * @return a new array which includes [slope, intercept]
	 */
	public static double[] calculateLineEquation(Translation2d firstPointOnLine, Translation2d secondPointOnLine) {
		double slope = (secondPointOnLine.getY() - firstPointOnLine.getY()) / (secondPointOnLine.getX() - firstPointOnLine.getX());
		double intercept = firstPointOnLine.getY() - slope * firstPointOnLine.getX();
		return new double[] {slope, intercept};
	}

	/**
	 * @param lineFunction should include [slope, intercept]
	 * @return the distance between the translation and the given line function
	 */
	public static double calculateTranslationDistanceFromLine(Translation2d translation, double[] lineFunction) {
		double lineSlope = lineFunction[0];
		double lineIntercept = lineFunction[1];

		double A = -lineSlope;
		double B = 1;
		double C = -lineIntercept;

		return Math.abs(A * translation.getX() + B * translation.getY() + C) / Math.sqrt(A * A + B * B);
	}

}
