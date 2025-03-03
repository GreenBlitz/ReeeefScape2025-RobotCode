package frc.utils.math;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

public class PoseMath {

	public static Translation2d getRelativeTranslation(Translation2d relativeTo, Translation2d toRelative) {
		return toRelative.minus(relativeTo);
	}

	/**
	 * @param line Should be < Start point, End point >
	 */
	public static Translation2d getClosestPointBetweenPointAndLine(Translation2d targetPoint, Pair<Translation2d, Translation2d> line) {
		Translation2d startPoint = line.getFirst();
		Translation2d endPoint = line.getSecond();
		double slope = (startPoint.getY() - endPoint.getY()) / (startPoint.getX() - endPoint.getX());
		if (slope == 0)
			return new Translation2d(targetPoint.getX(), startPoint.getY());
		double perpendicularSlope = -1 / slope;

		double yIntercept1 = targetPoint.getY() - (targetPoint.getX() * perpendicularSlope);
		double yIntercept2 = startPoint.getY() - (startPoint.getX() * slope);

		double xClosest = (yIntercept1 - yIntercept2) / (slope - perpendicularSlope);
		double yClosest = xClosest * slope + yIntercept2;
		return new Translation2d(xClosest, yClosest);
	}

}
