package frc.robot.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.constants.DirectoryPaths;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.VisionData;
import frc.robot.vision.sources.VisionSource;
import frc.robot.vision.sources.limelights.DynamicSwitchingLimelight;
import frc.utils.Filter;
import frc.utils.alerts.Alert;
import frc.utils.math.AngleUnit;

import java.io.IOException;
import java.util.List;

public class VisionConstants {

	public static final String FILTERED_DATA_LOGPATH_ADDITION = "FilteredData/";

	public static final String NON_FILTERED_DATA_LOGPATH_ADDITION = "NonFilteredData/";

	public static final String VISION_SOURCE_LOGPATH_ADDITION = "VisionSource/";

	public static final String MULTI_VISION_SOURCES_LOGPATH = "MultiVisionSources/";

	public static final String DYNAMIC_LIMELIGHT_MEGATAG1_SOURCE_NAME = "independentPoseEstimatingLimelight";

	public static final String DYNAMIC_LIMELIGHT_MEGATAG2_SOURCE_NAME = "headingRequiringLimelight";


	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = getAprilTagFieldLayout();

	private static AprilTagFieldLayout getAprilTagFieldLayout() {
		try {
			return AprilTagFieldLayout.loadFromResource(DirectoryPaths.APRIL_TAG_FIELD_CONFIG_FILE_PATH.toString());
		} catch (IOException ioException) {
			new Alert(
				Alert.AlertType.WARNING,
				"Cannot read april tag field layout from " + DirectoryPaths.APRIL_TAG_FIELD_CONFIG_FILE_PATH + ", using default field layout"
			).report();
			return AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
		}
	}

	public static final int LIMELIGHT_ENTRY_ARRAY_LENGTH = 6;

	public static final int NO_APRILTAG_ID = -1;

	public static Rotation2d ROLL_FILTER_TOLERANCE = Rotation2d.fromDegrees(3);

	public static Rotation2d PITCH_FILTER_TOLERANCE = Rotation2d.fromDegrees(3);

	public static Rotation2d YAW_FILTER_TOLERANCE = Rotation2d.fromDegrees(1);

	public static double ROBOT_POSITION_IN_FIELD_TOLERANCE_METERS = 0.1;

	public static double ROBOT_DISTANCE_TO_GROUND_TOLERANCE_METERS = 0.3;

	public static double DEFAULT_RATIO_BETWEEN_IMU_AND_SOURCE_LIMELIGHT_4 = 0.001;

	public static double LIMELIGHT_4_MAXIMUM_LIMELIGHT_TEMPERATURE_CELSIUS = 80;

	public static double LIMELIGHT_4_MAXIMUM_CPU_TEMPERATURE_CELSIUS = 80;

	public static boolean REGULATE_TEMPERATURE_IN_LIMELIGHT_4 = false;

	public static int FALLBACK_SKIPPED_FRAMES_LIMELIGHT_4 = 100;

	// solves heating issues on limelight 4. According to the official docs, 50-100 should resolve any heating issues.
	public static int DEFAULT_SKIPPED_FRAMES_LIMELIGHT_4 = 2;

	public static final Filter<VisionData> DEFAULT_VISION_FILTER = VisionFilters.isInField(ROBOT_POSITION_IN_FIELD_TOLERANCE_METERS)
		.and(VisionFilters.isRollAtAngle(Rotation2d.fromDegrees(0), ROLL_FILTER_TOLERANCE))
		.and(VisionFilters.isPitchAtAngle(Rotation2d.fromDegrees(0), PITCH_FILTER_TOLERANCE))
		.and(VisionFilters.isOnGround(ROBOT_DISTANCE_TO_GROUND_TOLERANCE_METERS));

	public static final Pose3d LIMELIGHT_LEFT_CAMERA_ROBOT_POSE = new Pose3d(
		new Translation3d(0.22989, -0.11998, 0.48927),
		AngleUnit.DEGREES.toRotation3d(-8.6, -27.07, -21.72)
	);

	public static final Pose3d LIMELIGHT_RIGHT_CAMERA_ROBOT_POSE = new Pose3d(
		new Translation3d(0.215, 0.11, 0.495),
		AngleUnit.DEGREES.toRotation3d(11.21, -24.65, 23.76)
	);

	public static final VisionSource<AprilTagVisionData> LIMELIGHT_LEFT = new DynamicSwitchingLimelight(
		true,
		"limelight-left",
		VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
		"limelight4-front",
		VisionConstants.DEFAULT_VISION_FILTER,
		LIMELIGHT_LEFT_CAMERA_ROBOT_POSE
	);

	public static final VisionSource<AprilTagVisionData> LIMELIGHT_RIGHT = new DynamicSwitchingLimelight(
		true,
		"limelight-right",
		VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
		"limelight3gb-front",
		VisionConstants.DEFAULT_VISION_FILTER,
		LIMELIGHT_RIGHT_CAMERA_ROBOT_POSE
	);

	public static final List<VisionSource<AprilTagVisionData>> VISION_SOURCES = List.of(LIMELIGHT_LEFT, LIMELIGHT_RIGHT);

}
