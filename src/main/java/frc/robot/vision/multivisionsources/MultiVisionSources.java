package frc.robot.vision.multivisionsources;

import frc.robot.vision.VisionConstants;
import frc.robot.vision.data.VisionData;
import frc.robot.vision.sources.VisionSource;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class MultiVisionSources<ReturnType extends VisionData> {

	private final String logPath;
	private final List<VisionSource<ReturnType>> visionSources;

	@SafeVarargs
	public MultiVisionSources(String logPath, VisionSource<ReturnType>... visionSources) {
		this.logPath = logPath;
		this.visionSources = List.of(visionSources);
	}

	public MultiVisionSources(String logPath, List<VisionSource<ReturnType>> visionSources) {
		this.logPath = logPath;
		this.visionSources = visionSources;
	}

	protected List<VisionSource<ReturnType>> getVisionSources() {
		return visionSources;
	}

	protected ArrayList<ReturnType> createMappedCopyOfSources(
		List<VisionSource<ReturnType>> list,
		Function<VisionSource<ReturnType>, Optional<ReturnType>> mapping
	) {
		ArrayList<ReturnType> output = new ArrayList<>();
		list.forEach(visionSource -> {
			visionSource.update();
			Optional<ReturnType> observation = mapping.apply(visionSource);
			observation.ifPresent(output::add);
		});
		return output;
	}

	public ArrayList<ReturnType> getUnfilteredVisionData() {
		return createMappedCopyOfSources(visionSources, VisionSource::getVisionData);
	}

	public ArrayList<ReturnType> getFilteredVisionData() {
		return createMappedCopyOfSources(visionSources, VisionSource::getFilteredVisionData);
	}

	private static <ReturnType extends VisionData> void logPoses(String logPath, String logPathSuffix, List<ReturnType> observations) {
		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(logPath + logPathSuffix + i, observations.get(i).getEstimatedPose());
		}
	}

	private void log() {
		logPoses(logPath, VisionConstants.FILTERED_DATA_LOGPATH_ADDITION, getFilteredVisionData());
		logPoses(logPath, VisionConstants.NON_FILTERED_DATA_LOGPATH_ADDITION, getUnfilteredVisionData());
	}

	public void periodic() {
		log();
	}

}
