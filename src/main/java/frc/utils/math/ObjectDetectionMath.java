package frc.utils.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.objectdetection.ObjectDetectionHelpers;


public class ObjectDetectionMath {

	public static Pair<Double, Double> pixelToTxAndTyRadians(
		Translation2d pixel,
		int pictureWidthPixels,
		int pictureHeightPixels,
		double cameraHorizontalFOVRadians,
		double cameraVerticalFOVRadians
	) {
		double normalisedX = (pixel.getX() - (pictureWidthPixels / 2.0)) / (pictureWidthPixels / 2.0);
		double normalisedY = ((pictureHeightPixels / 2.0) - pixel.getY()) / (pictureHeightPixels / 2.0);

		double viewPlaneWidthRadians = 2 * Math.tan(cameraHorizontalFOVRadians / 2);
		double viewPlaneHeightRadians = 2 * Math.tan(cameraVerticalFOVRadians / 2);

		Translation2d pointOnViewPlane = new Translation2d(
			normalisedX * (viewPlaneWidthRadians / 2),
			normalisedY * (viewPlaneHeightRadians / 2)
		);
		double tx = Math.atan(pointOnViewPlane.getX());
		double ty = Math.atan(pointOnViewPlane.getY());
		return new Pair<>(tx, ty);
	}

	public static double getObjectHeightToWidthRatio(double[] t2dEntryArray) {
		double detectedHorizontalPixels = t2dEntryArray[14];
		double detectedVerticalPixels = t2dEntryArray[15];
		return detectedVerticalPixels / detectedHorizontalPixels;
	}

	public static Translation2d getObjectCenterPixel(double[] rawDetectionsEntryArray, int firstCellIndex) {
		Translation2d[] objectFrameCorners = ObjectDetectionHelpers.getAllObjectFrameCorners(rawDetectionsEntryArray, firstCellIndex);
		double smallestFrameX = objectFrameCorners[0].getX();
		double centerXFromEdge = (objectFrameCorners[1].getX() - objectFrameCorners[0].getX()) / 2;
		double centerX = smallestFrameX + centerXFromEdge;

		double smallestFrameY = objectFrameCorners[0].getY();
		double centerYFromEdge = (objectFrameCorners[3].getY() - objectFrameCorners[0].getY()) / 2;
		double centerY = smallestFrameY + centerYFromEdge;

		return new Translation2d(centerX, centerY);
	}

	public static Translation2d findRealSquishedAlgaeCenter(
		Translation2d squishedCenterPixel,
		double algaeHeightToWidthRatio,
		int pictureWidthPixels,
		int pictureHeightPixels
	) {
		if (algaeHeightToWidthRatio > VisionConstants.ALGAE_HEIGHT_TO_WIDTH_RATIO) {
			return findRealXSquishedAlgaeCenter(squishedCenterPixel, algaeHeightToWidthRatio, pictureWidthPixels);
		} else {
			return findRealYSquishedAlgaeCenter(squishedCenterPixel, algaeHeightToWidthRatio, pictureHeightPixels);
		}
	}

	private static Translation2d findRealXSquishedAlgaeCenter(
		Translation2d squishedCenterPixel,
		double algaeHeightToWidthRatio,
		int pictureWidthPixels
	) {
		int squishedCenterX = (int) squishedCenterPixel.getX();
		double realCenterX;

		if (squishedCenterX > (pictureWidthPixels / 2)) {
			int objectFrameWidthPixels = 2 * (pictureWidthPixels - squishedCenterX);
			int verticalFrameEdgeSmallestX = pictureWidthPixels - objectFrameWidthPixels;
			realCenterX = (((double) objectFrameWidthPixels / 2) * algaeHeightToWidthRatio) + verticalFrameEdgeSmallestX;
		} else {
			int objectFrameWidthPixels = 2 * squishedCenterX;
			realCenterX = objectFrameWidthPixels - (squishedCenterX * algaeHeightToWidthRatio);
		}

		return new Translation2d(realCenterX, squishedCenterPixel.getY());
	}

	private static Translation2d findRealYSquishedAlgaeCenter(
		Translation2d squishedCenterPixel,
		double algaeHeightToWidthRatio,
		int pictureHeightPixels
	) {
		int squishedCenterY = (int) squishedCenterPixel.getY();
		double realCenterY;

		if (squishedCenterY > (pictureHeightPixels / 2)) {
			int objectFrameHeightPixels = 2 * (pictureHeightPixels - squishedCenterY);
			int horizontalFrameEdgeSmallestY = pictureHeightPixels - objectFrameHeightPixels;
			realCenterY = (((double) objectFrameHeightPixels / 2) * (1 / algaeHeightToWidthRatio)) + horizontalFrameEdgeSmallestY;
		} else {
			int objectFrameHeightPixels = 2 * squishedCenterY;
			realCenterY = objectFrameHeightPixels - (squishedCenterY * (1 / algaeHeightToWidthRatio));
		}

		return new Translation2d(squishedCenterPixel.getX(), realCenterY);
	}

	public static boolean isPixelOnEdgeOfPicture(Translation2d pixel, int pictureWidthPixels, int pictureHeightPixels, int tolerance) {
		return MathUtil.isNear(0, pixel.getX(), tolerance)
			|| MathUtil.isNear(pictureWidthPixels, pixel.getX(), tolerance)
			|| MathUtil.isNear(0, pixel.getY(), tolerance)
			|| MathUtil.isNear(pictureHeightPixels, pixel.getY(), tolerance);
	}

	public static Pair<Rotation2d, Rotation2d> correctForCameraRoll(Rotation2d yaw, Rotation2d pitch, Pose3d cameraPose) {
		Translation2d yawAndPitch = new Translation2d(yaw.getRadians(), pitch.getRadians());

//		Rotation3d cameraRotation = cameraPose.getRotation().rotateBy(new Rotation3d(0, 0, -cameraPose.getRotation().getZ()));
//		double rollRadians = cameraRotation.rotateBy(new Rotation3d(0, -cameraRotation.getY(), 0)).getX();
		double rollRadians = cameraPose.getRotation().getX();

		yawAndPitch = yawAndPitch.rotateBy(Rotation2d.fromRadians(rollRadians).unaryMinus());
		return new Pair<>(Rotation2d.fromRadians(yawAndPitch.getX()), Rotation2d.fromRadians(yawAndPitch.getY()));
	}

	public static double getCameraRelativeXDistance(Rotation2d cameraRelativePitch, Pose3d cameraPose, double centerOfObjectHeightMeters) {
//		double cameraPitchRadians = cameraPose.getRotation().rotateBy(new Rotation3d(0, 0, -cameraPose.getRotation().getZ())).getY();
		double cameraPitchRadians = cameraPose.getRotation().getY();
		Rotation2d pitch = cameraRelativePitch.plus(Rotation2d.fromRadians(cameraPitchRadians));

		double heightMeters = centerOfObjectHeightMeters - cameraPose.getZ();
		return heightMeters / pitch.getTan();
	}

	public static double getCameraRelativeYDistance(
		Rotation2d cameraRelativeYaw,
		double XDistanceMeters,
		Pose3d cameraPose,
		double centerOfObjectHeightMeters
	) {
		double heightMeters = centerOfObjectHeightMeters - cameraPose.getZ();
		double cameraToObjectXAxisHypotenuseMeters = Math.hypot(XDistanceMeters, heightMeters);

//		tx (represented by cameraRelativeYaw) is flipped (unaryMinus) because of x-axis positive direction conventions
		return cameraRelativeYaw.unaryMinus().getTan() * cameraToObjectXAxisHypotenuseMeters;
	}

	public static Translation2d cameraRelativeToRobotRelative(Translation2d translation, Pose3d cameraPose) {
		translation = translation.rotateBy(Rotation2d.fromRadians(cameraPose.getRotation().getZ()));
		return new Translation2d(translation.getX() + cameraPose.getX(), translation.getY() + cameraPose.getY());
	}

	public static Translation2d getRobotRelativeTranslation(
		Rotation2d cameraRollRelativeYaw,
		Rotation2d cameraRollRelativePitch,
		Pose3d cameraPose,
		double centerOfObjectHeightMeters
	) {
//		Pair<Rotation2d, Rotation2d> correctedYawAndPitch = correctForCameraRoll(cameraRollRelativeYaw, cameraRollRelativePitch, cameraPose);
		Pair<Rotation2d, Rotation2d> correctedYawAndPitch = new Pair<>(cameraRollRelativeYaw, cameraRollRelativePitch);

		double xDistance = getCameraRelativeXDistance(correctedYawAndPitch.getSecond(), cameraPose, centerOfObjectHeightMeters);
		double yDistance = getCameraRelativeYDistance(correctedYawAndPitch.getFirst(), xDistance, cameraPose, centerOfObjectHeightMeters);
		Translation2d cameraRelativeTranslation = new Translation2d(xDistance, yDistance);
		return cameraRelativeToRobotRelative(cameraRelativeTranslation, cameraPose);
	}

}
