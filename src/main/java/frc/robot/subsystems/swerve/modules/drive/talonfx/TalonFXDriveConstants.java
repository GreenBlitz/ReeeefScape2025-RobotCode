package frc.robot.subsystems.swerve.modules.drive.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.subsystems.swerve.modules.drive.DriveConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXDriveConstants {

	static final int APPLY_CONFIG_RETRIES = 10;

	private final TalonFXWrapper motor;
	private final TalonFXDriveSignals signals;
	private final boolean enableFOC;

	public TalonFXDriveConstants(
		CTREDeviceID motorID,
		boolean inverted,
		TalonFXConfiguration configuration,
		boolean enableFOC,
		String logPathPrefix
	) {
		TalonFXDriveConfigObject talonFXDriveConfigObject = new TalonFXDriveConfigObject(
			motorID,
			inverted,
			configuration,
			logPathPrefix + DriveConstants.LOG_PATH_ADDITION
		);
		this.motor = talonFXDriveConfigObject.getMotor();
		this.signals = talonFXDriveConfigObject.getSignals();
		this.enableFOC = enableFOC;
	}

	protected TalonFXWrapper getMotor() {
		return motor;
	}

	protected TalonFXDriveSignals getSignals() {
		return signals;
	}

	protected boolean getEnableFOC() {
		return enableFOC;
	}

}
