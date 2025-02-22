package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.robot.vision.sources.limelights.limelight4.LimelightIMUMode;
import frc.robot.vision.sources.limelights.limelight4.LimeLight4;
import frc.utils.Filter;

public class LimelightFactory {

	public static RobotHeadingRequiringVisionSource createRobotHeadingRequiringLimelight(
		String cameraNetworkTablesName,
		String parentLogPath,
		String sourceName,
		Filter<? super AprilTagVisionData> filter,
		Pose3d cameraPoseOffset
	) {
		return new LimeLightSource(
			cameraNetworkTablesName,
			parentLogPath,
			sourceName,
			filter,
			cameraPoseOffset,
			LimelightPoseEstimationMethod.MEGATAG_2
		);
	}

	public static IndpendentHeadingVisionSource createRobotHeadingEstimatingLimelight(
		String cameraNetworkTablesName,
		String parentLogPath,
		String sourceName,
		Filter<? super AprilTagVisionData> filter,
		Pose3d cameraPoseOffset
	) {
		return new LimeLightSource(
			cameraNetworkTablesName,
			parentLogPath,
			sourceName,
			filter,
			cameraPoseOffset,
			LimelightPoseEstimationMethod.MEGATAG_1
		);
	}

	public static LimeLight4 createLimeLight4(
		String cameraNetworkTablesName,
		String parentLogPath,
		String sourceName,
		Filter<? super AprilTagVisionData> filter,
		Pose3d cameraPoseOffset,
		LimelightPoseEstimationMethod poseEstimationMethod
	) {
		return new LimeLight4(
			cameraNetworkTablesName,
			parentLogPath,
			sourceName,
			filter,
			cameraPoseOffset,
			poseEstimationMethod,
			LimelightIMUMode.USE_INTERNAL_ASSIST_MEGATAG_1,
			VisionConstants.DEFAULT_SKIPPED_FRAMES_LIMELIGHT_4,
			VisionConstants.DEFAULT_RATIO_BETWEEN_IMU_AND_SOURCE_LIMELIGHT_4
		);
	}

}
