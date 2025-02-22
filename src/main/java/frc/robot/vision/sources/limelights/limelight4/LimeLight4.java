package frc.robot.vision.sources.limelights.limelight4;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
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

public class LimeLight4 extends LimeLightSource implements IndpendentHeadingVisionSource, RobotHeadingRequiringVisionSource {

	private final NetworkTableEntry mutableIMUDataEntry;
	private final NetworkTableEntry mutableIMUModeEntry;
	private final NetworkTableEntry mutableFramesToSkipEntry;
	private final NetworkTableEntry mutableInternalIMURelianceEntry;

	private IMUMode imuMode;
	private double[] imuData;

	public LimeLight4(
		String cameraNetworkTablesName,
		String parentLogPath,
		String sourceName,
		Filter<? super AprilTagVisionData> filter,
		Pose3d cameraPoseOffset,
		LimelightPoseEstimationMethod poseEstimationMethod,
		IMUMode defaultIMUMode,
		int defaultSkippedFramesCount,
		double defaultRatioBetweenIMUAndSource
	) {
		super(cameraNetworkTablesName, parentLogPath, sourceName, filter, cameraPoseOffset, poseEstimationMethod);
		this.mutableIMUDataEntry = getLimelightNetworkTableEntry("imu");
		this.mutableIMUModeEntry = getLimelightNetworkTableEntry("imumode_set");
		this.mutableFramesToSkipEntry = getLimelightNetworkTableEntry("throttle_set");
		this.mutableInternalIMURelianceEntry = getLimelightNetworkTableEntry("imuassistalpha_set");
		setIMUMode(defaultIMUMode);
		setSkippedFramesProcessing(defaultSkippedFramesCount);
		setInternalIMUReliance(defaultRatioBetweenIMUAndSource);
	}

	public void setIMUMode(IMUMode imuMode) {
		this.imuMode = imuMode;
		mutableIMUModeEntry.setInteger(imuMode.getAPIValue());
	}

	public IMUMode getIMMode() {
		return imuMode;
	}

	public void setSkippedFramesProcessing(int framesCount) {
		mutableFramesToSkipEntry.setInteger(framesCount);
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
		imuData = mutableIMUDataEntry.getDoubleArray(new double[IMUDataLimelight.values().length]);
	}

	@Override
	public Optional<TimedValue<Rotation2d>> getRawHeadingData() {
		return imuMode.isIndependent()
			? Optional
				.of(new TimedValue<>(Rotation2d.fromRadians(imuData[IMUDataLimelight.ROBOT_YAW.getIndex()]), TimeUtil.getCurrentTimeSeconds()))
			: Optional.empty();
	}

	public double getAccelerationX() {
		return imuData[IMUDataLimelight.ACCELERATION_X.getIndex()];
	}

	public double getAccelerationY() {
		return imuData[IMUDataLimelight.ACCELERATION_Y.getIndex()];
	}

	public double getAccelerationZ() {
		return imuData[IMUDataLimelight.ACCELERATION_Z.getIndex()];
	}

	public double getYaw() {
		return imuData[IMUDataLimelight.YAW.getIndex()];
	}

	public double getPitch() {
		return imuData[IMUDataLimelight.PITCH.getIndex()];
	}

	public double getRoll() {
		return imuData[IMUDataLimelight.ROLL.getIndex()];
	}

	public double getGyroX() {
		return imuData[IMUDataLimelight.GYRO_X.getIndex()];
	}

	public double getGyroY() {
		return imuData[IMUDataLimelight.GYRO_Y.getIndex()];
	}

	public double getGyroZ() {
		return imuData[IMUDataLimelight.GYRO_Z.getIndex()];
	}

	@Override
	public void log() {
		super.log();
		Logger.recordOutput(logPath + "IMUMode", imuMode);

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
