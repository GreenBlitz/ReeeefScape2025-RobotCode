package frc.robot.subsystems.swerve.module;

import frc.utils.battery.BatteryUtil;

public class ModuleConstants {

	public static final double VOLTAGE_COMPENSATION_SATURATION = BatteryUtil.DEFAULT_VOLTAGE;
	public static final boolean DEFAULT_IS_CLOSE_LOOP = true;
	public static final String MODULES_LOG_PATH_ADDITION = "/Modules";
	public static final int NUMBER_OF_DRIVE_MOTORS = 1;
	public static final double COEFFICIENT_OF_FRICTION = 0.96;
	public static final double MAXIMUM_MODULE_ROTATIONAL_SPEED_RADIANS_PER_SECOND = edu.wpi.first.math.util.Units.rotationsToRadians(7); // TODO:
																																			// calibrate

}
