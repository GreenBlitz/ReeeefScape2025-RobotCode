package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import frc.robot.vision.VisionConstants;
import frc.utils.TimedValue;
import frc.robot.vision.RobotAngleValues;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.utils.Filter;

import java.util.Optional;

/**
 * <code>DynamicSwitchingLimelight</code> is a class that automatically handles the switching between megatag 1 and megatag 2. It means it can
 * update the limelight, give the independent heading from the correct entry, switch between megatag 1 and 2 on runtime, and more.
 * <code>MultiAprilTagVisionSources</code> can make use of special methods of this class.
 */
public class DynamicSwitchingLimelight implements IndpendentHeadingVisionSource, RobotHeadingRequiringVisionSource {

	private final IndpendentHeadingVisionSource independentPoseEstimatingLimelight;
	private final RobotHeadingRequiringVisionSource headingRequiringLimelight;
	private boolean useGyroForPoseEstimating;

	public DynamicSwitchingLimelight(
		boolean defaultUseGyroForPoseEstimating,
		String cameraNetworkTablesName,
		String parentLogPath,
		String sourceName,
		Filter<? super AprilTagVisionData> headingRequiringLimelightLimelightFilter,
		Filter<? super AprilTagVisionData> indapdendentLimelightLimelightFilter,
		Pose3d cameraPoseOffset
	) {
		this.useGyroForPoseEstimating = defaultUseGyroForPoseEstimating;
		this.independentPoseEstimatingLimelight = LimelightFactory.createRobotHeadingEstimatingLimelight(
			cameraNetworkTablesName,
			parentLogPath,
			sourceName + "/" + VisionConstants.DYNAMIC_LIMELIGHT_MEGATAG1_SOURCE_NAME,
			indapdendentLimelightLimelightFilter,
			cameraPoseOffset
		);
		this.headingRequiringLimelight = LimelightFactory.createRobotHeadingRequiringLimelight(
			cameraNetworkTablesName,
			parentLogPath,
			sourceName + "/" + VisionConstants.DYNAMIC_LIMELIGHT_MEGATAG2_SOURCE_NAME,
			headingRequiringLimelightLimelightFilter,
			cameraPoseOffset
		);
	}

	public void setUseRobotHeadingForPoseEstimating(boolean useGyroForPoseEstimating) {
		this.useGyroForPoseEstimating = useGyroForPoseEstimating;
	}

	@Override
	public void update() {
		independentPoseEstimatingLimelight.update();
		headingRequiringLimelight.update();
	}

	@Override
	public Optional<AprilTagVisionData> getVisionData() {
		return useGyroForPoseEstimating ? headingRequiringLimelight.getVisionData() : independentPoseEstimatingLimelight.getVisionData();
	}

	@Override
	public Optional<AprilTagVisionData> getFilteredVisionData() {
		return useGyroForPoseEstimating
			? headingRequiringLimelight.getFilteredVisionData()
			: independentPoseEstimatingLimelight.getFilteredVisionData();
	}

	/**
	 * set the filter for both of the LimeLight.
	 * @param newFilter: the new filter to be used in both LimeLight.
	 */
	@Override
	public void setFilter(Filter<? super AprilTagVisionData> newFilter) {
		setFilter(newFilter, newFilter);
	}

	public void setFilter(Filter<? super AprilTagVisionData> indapendentLimelightFilter, Filter<? super AprilTagVisionData> headingRequiringLimelightLimelightFilter) {
		independentPoseEstimatingLimelight.setFilter(indapendentLimelightFilter);
		headingRequiringLimelight.setFilter(headingRequiringLimelightLimelightFilter);
	}

	/**
	 * @return returns a stronger combination version of both filters.
	 */
	@Override
	public Filter<? super AprilTagVisionData> getFilter() {
		return new Filter<AprilTagVisionData>((value) -> independentPoseEstimatingLimelight.getFilter().apply(value) && headingRequiringLimelight.getFilter().apply(value));
	}

	public Filter<? super AprilTagVisionData> getIndapdendentLimelightLimelightFilter() {
		return independentPoseEstimatingLimelight.getFilter();
	}

	public Filter<? super AprilTagVisionData> getHeadingRequiringLimelightLimelightFilter() {
		return headingRequiringLimelight.getFilter();
	}

	@Override
	public Optional<TimedValue<Rotation2d>> getRawHeadingData() {
		return independentPoseEstimatingLimelight.getRawHeadingData();
	}

	@Override
	public Optional<TimedValue<Rotation2d>> getFilteredHeadingData() {
		return independentPoseEstimatingLimelight.getFilteredHeadingData();
	}

	@Override
	public void updateRobotAngleValues(RobotAngleValues robotAngleValues) {
		headingRequiringLimelight.updateRobotAngleValues(robotAngleValues);
	}

}
