package frc.robot.vision.objectdetection;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.data.ObjectData;
import frc.utils.Filter;
import frc.utils.math.ObjectDetectionMath;
import frc.utils.time.TimeUtil;

import java.util.Optional;

public class ObjectDetectionHelpers {

	public static Pair<Double, Double> txNoCrossAndTyNoCrossToTxAndTy(double txNoCross, double tyNoCross) {
		double tx = txNoCross - (VisionConstants.LIMELIGHT_3_HORIZONTAL_FOV.getDegrees() / 2);
		double ty = (VisionConstants.LIMELIGHT_3_VERTICAL_FOV.getDegrees() / 2) - tyNoCross;
		return new Pair<>(tx, ty);
	}

	public static Optional<Integer> getObjectsFirstCellIndexInRawDetectionsArray(
		double txDegrees,
		double tyDegrees,
		double[] rawDetectionsEntryArray
	) {
		int objectAmount = rawDetectionsEntryArray.length / VisionConstants.OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY;

		for (int i = 0; i < objectAmount; i++) {
			int firstCell = VisionConstants.OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY * i;
			double rawDetectionsEntryTxNoCrossValue = rawDetectionsEntryArray[firstCell + RawDetectionsEntryIndexes.TX_NO_CROSS.ordinal()];
			double rawDetectionsEntryTyNoCrossValue = rawDetectionsEntryArray[firstCell + RawDetectionsEntryIndexes.TY_NO_CROSS.ordinal()];

			Pair<Double, Double> rawDetectionsEntryTxAndTyValues = txNoCrossAndTyNoCrossToTxAndTy(
				rawDetectionsEntryTxNoCrossValue,
				rawDetectionsEntryTyNoCrossValue
			);

			if (rawDetectionsEntryTxAndTyValues.getFirst() == txDegrees && rawDetectionsEntryTxAndTyValues.getSecond() == tyDegrees) {
				return Optional.of(firstCell);
			}
		}
		return Optional.empty();
	}

	public static int getNumberOfObjectCornersOnPictureEdge(
		double[] rawDetectionsEntryArray,
		int objectFirstCellIndex,
		int pictureWidthPixels,
		int pictureHeightPixels,
		int edgePixelTolerance
	) {
		int numberOfCornersOnPictureEdge = 0;
		Translation2d[] objectFrameCorners = getAllObjectFrameCorners(rawDetectionsEntryArray, objectFirstCellIndex);

		for (Translation2d corner : objectFrameCorners) {
			if (ObjectDetectionMath.isPixelOnEdgeOfPicture(corner, pictureWidthPixels, pictureHeightPixels, edgePixelTolerance)) {
				numberOfCornersOnPictureEdge++;
			}
		}
		return numberOfCornersOnPictureEdge;
	}

	public static Translation2d[] getAllObjectFrameCorners(double[] rawDetectionsEntryArray, int objectFirstCellIndex) {
		return new Translation2d[] {
			new Translation2d(
				rawDetectionsEntryArray[objectFirstCellIndex + RawDetectionsEntryIndexes.TOP_LEFT_CORNER_X.ordinal()],
				rawDetectionsEntryArray[objectFirstCellIndex + RawDetectionsEntryIndexes.TOP_LEFT_CORNER_Y.ordinal()]
			),
			new Translation2d(
				rawDetectionsEntryArray[objectFirstCellIndex + RawDetectionsEntryIndexes.TOP_RIGHT_CORNER_X.ordinal()],
				rawDetectionsEntryArray[objectFirstCellIndex + RawDetectionsEntryIndexes.TOP_RIGHT_CORNER_Y.ordinal()]
			),
			new Translation2d(
				rawDetectionsEntryArray[objectFirstCellIndex + RawDetectionsEntryIndexes.BOTTOM_RIGHT_CORNER_X.ordinal()],
				rawDetectionsEntryArray[objectFirstCellIndex + RawDetectionsEntryIndexes.BOTTOM_RIGHT_CORNER_Y.ordinal()]
			),
			new Translation2d(
				rawDetectionsEntryArray[objectFirstCellIndex + RawDetectionsEntryIndexes.BOTTOM_LEFT_CORNER_X.ordinal()],
				rawDetectionsEntryArray[objectFirstCellIndex + RawDetectionsEntryIndexes.BOTTOM_LEFT_CORNER_Y.ordinal()]
			)};
	}

	public static Optional<Pair<Double, Double>> getSquishedAlgaeRealTxAndTyRadians(
		double txDegrees,
		double tyDegrees,
		Filter<double[]> t2dEntrySquishedAlgaeFilter,
		double[] t2dEntryArray,
		double[] rawDetectionsEntryArray
	) {
		Optional<Integer> objectFirstCellIndexInRawDetectionsArray = getObjectsFirstCellIndexInRawDetectionsArray(
			txDegrees,
			tyDegrees,
			rawDetectionsEntryArray
		);
		if (objectFirstCellIndexInRawDetectionsArray.isEmpty()) {
			return Optional.empty();
		}

		Translation2d algaeCenterPixel = ObjectDetectionMath
			.getObjectCenterPixel(rawDetectionsEntryArray, objectFirstCellIndexInRawDetectionsArray.get());
		double algaeHeightToWidthRatio = ObjectDetectionMath.getObjectHeightToWidthRatio(t2dEntryArray);

		boolean isAlgaeSquished = !t2dEntrySquishedAlgaeFilter.apply(t2dEntryArray);
		boolean isAlgaeCutOffOnPictureCorner = ObjectDetectionHelpers.getNumberOfObjectCornersOnPictureEdge(
			rawDetectionsEntryArray,
			objectFirstCellIndexInRawDetectionsArray.get(),
			(int) VisionConstants.LIMELIGHT_OBJECT_RESOLUTION_PIXELS.getX(),
			(int) VisionConstants.LIMELIGHT_OBJECT_RESOLUTION_PIXELS.getY(),
			VisionConstants.EDGE_PIXEL_TOLERANCE
		) >= 3;

		if (!isAlgaeSquished && !isAlgaeCutOffOnPictureCorner) {
			return Optional.of(new Pair<>(txDegrees, tyDegrees));
		}

		if (isAlgaeSquished && !isAlgaeCutOffOnPictureCorner) {
			Translation2d realCenterPixel = ObjectDetectionMath.findRealSquishedAlgaeCenter(
				algaeCenterPixel,
				algaeHeightToWidthRatio,
				(int) VisionConstants.LIMELIGHT_OBJECT_RESOLUTION_PIXELS.getX(),
				(int) VisionConstants.LIMELIGHT_OBJECT_RESOLUTION_PIXELS.getY()
			);
			return Optional.of(
				ObjectDetectionMath.pixelToTxAndTyRadians(
					realCenterPixel,
					(int) VisionConstants.LIMELIGHT_OBJECT_RESOLUTION_PIXELS.getX(),
					(int) VisionConstants.LIMELIGHT_OBJECT_RESOLUTION_PIXELS.getY(),
					VisionConstants.LIMELIGHT_3_HORIZONTAL_FOV.getRadians(),
					VisionConstants.LIMELIGHT_3_VERTICAL_FOV.getRadians()
				)
			);
		}

		return Optional.empty();
	}

	public static Filter<double[]> squishedAlgaeFilter(double algaeHeightToWidthRatio, double heightToWidthRatioTolerance) {
		return (t2dEntryArray) -> {
			double detectedHeightToWidthRatio = ObjectDetectionMath.getObjectHeightToWidthRatio(t2dEntryArray);
			return MathUtil.isNear(algaeHeightToWidthRatio, detectedHeightToWidthRatio, heightToWidthRatioTolerance);
		};
	}

	public static Optional<ObjectType> getObjectType(NetworkTableEntry objectNameEntry) {
		String nameEntryValue = objectNameEntry.getString(VisionConstants.NAME_ENTRY_NO_OBJECT_VALUE);
		for (ObjectType type : ObjectType.values()) {
			if (type.getNameEntryValue().equals(nameEntryValue)) {
				return Optional.of(type);
			}
		}
		return Optional.empty();
	}

	public static ObjectData getObjectData(
		double txDegrees,
		double tyDegrees,
		double pipelineLatency,
		double captureLatency,
		ObjectType objectType,
		Pose3d cameraPose
	) {
		double centerOfObjectHeightMeters = objectType.getObjectHeightMeters() / 2;
		Rotation2d cameraRelativeObjectYaw = Rotation2d.fromDegrees(txDegrees);
		Rotation2d cameraRelativeObjectPitch = Rotation2d.fromDegrees(tyDegrees);

		Translation2d robotRelativeObjectTranslation = ObjectDetectionMath
			.getRobotRelativeTranslation(cameraRelativeObjectYaw, cameraRelativeObjectPitch, cameraPose, centerOfObjectHeightMeters);

		double totalLatency = pipelineLatency + captureLatency;
		double timeStamp = TimeUtil.getCurrentTimeSeconds() - totalLatency;

		return new ObjectData(robotRelativeObjectTranslation, objectType, timeStamp);
	}

	public static ObjectData getObjectData(
		NetworkTableEntry txEntry,
		NetworkTableEntry tyEntry,
		NetworkTableEntry pipelineLatencyEntry,
		NetworkTableEntry captureLatencyEntry,
		ObjectType objectType,
		Pose3d cameraPose
	) {
		return getObjectData(
			txEntry.getDouble(0),
			tyEntry.getDouble(0),
			pipelineLatencyEntry.getDouble(0),
			captureLatencyEntry.getDouble(0),
			objectType,
			cameraPose
		);
	}

	public static Optional<ObjectData> getFilteredAlgaeObjectData(
		NetworkTableEntry txEntry,
		NetworkTableEntry tyEntry,
		NetworkTableEntry t2dEntry,
		NetworkTableEntry rawDetectionsEntry,
		NetworkTableEntry pipelineLatencyEntry,
		NetworkTableEntry captureLatencyEntry,
		Pose3d cameraPose
	) {
		Optional<Pair<Double, Double>> filteredTxAndTy = getSquishedAlgaeRealTxAndTyRadians(
			txEntry.getDouble(0),
			tyEntry.getDouble(0),
			squishedAlgaeFilter(VisionConstants.ALGAE_HEIGHT_TO_WIDTH_RATIO, VisionConstants.ALGAE_HEIGHT_TO_WIDTH_RATIO_TOLERANCE),
			t2dEntry.getDoubleArray(new double[0]),
			rawDetectionsEntry.getDoubleArray(new double[0])
		);

		if (filteredTxAndTy.isEmpty()) {
			return Optional.empty();
		}

		return Optional.of(
			getObjectData(
				filteredTxAndTy.get().getFirst(),
				filteredTxAndTy.get().getSecond(),
				pipelineLatencyEntry.getDouble(0),
				captureLatencyEntry.getDouble(0),
				ObjectType.ALGAE,
				cameraPose
			)
		);
	}

}
