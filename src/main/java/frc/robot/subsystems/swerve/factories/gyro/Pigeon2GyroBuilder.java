package frc.robot.subsystems.swerve.factories.gyro;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.gyro.Pigeon2Gyro;
import frc.robot.hardware.phoenix6.gyro.Pigeon2Wrapper;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.GyroSignals;
import frc.utils.alerts.Alert;
import frc.utils.math.AngleUnit;

class Pigeon2GyroBuilder {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private static Pigeon2Configuration buildGyroConfig(Pigeon2Wrapper gyro) {
		MountPoseConfigs mountPoseConfigs = new MountPoseConfigs();
		gyro.getConfigurator().refresh(mountPoseConfigs);

		mountPoseConfigs.MountPoseYaw = 180;

		Pigeon2Configuration gyroConfig = new Pigeon2Configuration();
		gyroConfig.MountPose = mountPoseConfigs;

		return gyroConfig;
	}

	static IGyro buildGyro(String logPath) {
		Pigeon2Wrapper pigeon2Wrapper = new Pigeon2Wrapper(IDs.SWERVE_PIGEON_2);
		if (!pigeon2Wrapper.applyConfiguration(buildGyroConfig(pigeon2Wrapper), APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailAt").report();
		}
		return new Pigeon2Gyro(logPath, pigeon2Wrapper);
	}

	static GyroSignals buildSignals(Pigeon2Gyro pigeon2Gyro) {
		return new GyroSignals(
			Phoenix6SignalBuilder.build(pigeon2Gyro.getDevice().getYaw(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.DEGREES)
		);
	}

}
