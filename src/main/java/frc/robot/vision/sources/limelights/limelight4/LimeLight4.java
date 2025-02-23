package frc.robot.vision.sources.limelights.limelight4;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.robot.vision.sources.limelights.LimeLightSource;
import frc.robot.vision.sources.limelights.LimelightPoseEstimationMethod;
import frc.utils.Filter;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static frc.robot.vision.VisionConstants.FALLBACK_SKIPPED_FRAMES_LIMELIGHT_4;

public class LimeLight4 extends LimeLightSource implements IndpendentHeadingVisionSource, RobotHeadingRequiringVisionSource {

	private final NetworkTableEntry mutableIMUDataEntry;
	private final NetworkTableEntry mutableIMUModeEntry;
	private final NetworkTableEntry mutableFramesToSkipEntry;
	private final NetworkTableEntry mutableInternalIMURelianceEntry;
	private final boolean regulateTemperature;

	private int lastManuallySetSkippedFrames;
	private LimelightIMUMode limelightImuMode;
	private double[] imuDataArray;

	public LimeLight4(
		String cameraNetworkTablesName,
		String parentLogPath,
		String sourceName,
		Filter<? super AprilTagVisionData> filter,
		Pose3d cameraPoseOffset,
		LimelightPoseEstimationMethod poseEstimationMethod,
		LimelightIMUMode defaultLimelightIMUMode,
		int defaultSkippedFramesCount,
		double defaultRatioBetweenIMUAndSource,
		boolean regulateTemperature
	) {
		super(cameraNetworkTablesName, parentLogPath, sourceName, filter, cameraPoseOffset, poseEstimationMethod);
		this.mutableIMUDataEntry = getLimelightNetworkTableEntry("imu");
		this.mutableIMUModeEntry = getLimelightNetworkTableEntry("imumode_set");
		this.mutableFramesToSkipEntry = getLimelightNetworkTableEntry("throttle_set");
		this.mutableInternalIMURelianceEntry = getLimelightNetworkTableEntry("imuassistalpha_set");
		this.regulateTemperature = regulateTemperature;
		setIMUMode(defaultLimelightIMUMode);
		setSkippedFramesProcessing(defaultSkippedFramesCount);
		setInternalIMUReliance(defaultRatioBetweenIMUAndSource);
	}

	public void setIMUMode(LimelightIMUMode limelightImuMode) {
		this.limelightImuMode = limelightImuMode;
		mutableIMUModeEntry.setInteger(limelightImuMode.getAPIValue());
	}

	public LimelightIMUMode getIMMode() {
		return limelightImuMode;
	}

	public void setSkippedFramesProcessing(int framesCount) {
		mutableFramesToSkipEntry.setInteger(framesCount);
		this.lastManuallySetSkippedFrames = framesCount;
	}

	public int getSkippedFramesProcessing() {
		return (int) mutableFramesToSkipEntry.getInteger(-1);
	}

	public void setInternalIMUReliance(double ratioBetweenIMUAndSource) {
		mutableInternalIMURelianceEntry.setDouble(ratioBetweenIMUAndSource);
	}

	public double getInternalIMUReliance() {
		return mutableInternalIMURelianceEntry.getDouble(-1);
	}

	@Override
	public void update() {
		super.update();
		if (regulateTemperature) {
			regulateTemperature();
		}
		imuDataArray = mutableIMUDataEntry.getDoubleArray(new double[LimeLightIMUData.length]);
	}

	@Override
	public Optional<TimedValue<Rotation2d>> getRawHeadingData() {
		return limelightImuMode.isIndependent()
			? Optional.of(
				new TimedValue<>(Rotation2d.fromRadians(imuDataArray[LimeLightIMUData.ROBOT_YAW.getIndex()]), TimeUtil.getCurrentTimeSeconds())
			)
			: Optional.empty();
	}

	public void regulateTemperature() {
		if (
			getLimeLightTemperature() > VisionConstants.LIMELIGHT_4_MAXIMUM_LIMELIGHT_TEMPERATURE_CELSIUS
				|| getCPUTemperature() > VisionConstants.LIMELIGHT_4_MAXIMUM_CPU_TEMPERATURE_CELSIUS
		) {
			mutableFramesToSkipEntry.setInteger(FALLBACK_SKIPPED_FRAMES_LIMELIGHT_4);
			Logger.recordOutput(logPath + "temperatureRegulation", "high");
		} else {
			mutableFramesToSkipEntry.setInteger(lastManuallySetSkippedFrames);
			Logger.recordOutput(logPath + "temperatureRegulation", "fine");
		}
	}

	public double getAccelerationX() {
		return imuDataArray[LimeLightIMUData.ACCELERATION_X.getIndex()];
	}

	public double getAccelerationY() {
		return imuDataArray[LimeLightIMUData.ACCELERATION_Y.getIndex()];
	}

	public double getAccelerationZ() {
		return imuDataArray[LimeLightIMUData.ACCELERATION_Z.getIndex()];
	}

	public double getYaw() {
		return imuDataArray[LimeLightIMUData.YAW.getIndex()];
	}

	public double getPitch() {
		return imuDataArray[LimeLightIMUData.PITCH.getIndex()];
	}

	public double getRoll() {
		return imuDataArray[LimeLightIMUData.ROLL.getIndex()];
	}

	public double getGyroX() {
		return imuDataArray[LimeLightIMUData.GYRO_X.getIndex()];
	}

	public double getGyroY() {
		return imuDataArray[LimeLightIMUData.GYRO_Y.getIndex()];
	}

	public double getGyroZ() {
		return imuDataArray[LimeLightIMUData.GYRO_Z.getIndex()];
	}

	@Override
	public void log() {
		super.log();
		Logger.recordOutput(logPath + "IMUMode", limelightImuMode);

		Logger.recordOutput(logPath + "acceleration/x", getAccelerationX());
		Logger.recordOutput(logPath + "acceleration/y", getAccelerationY());
		Logger.recordOutput(logPath + "acceleration/z", getAccelerationZ());

		Logger.recordOutput(logPath + "angles/yaw", getYaw());
		Logger.recordOutput(logPath + "angles/pitch", getPitch());
		Logger.recordOutput(logPath + "angles/roll", getRoll());

		Logger.recordOutput(logPath + "gyro/x", getGyroX());
		Logger.recordOutput(logPath + "gyro/y", getGyroY());
		Logger.recordOutput(logPath + "gyro/z", getGyroZ());
	}

}
