package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.gyro.Pigeon2Gyro;
import frc.robot.subsystems.swerve.IMUSignals;

public class GyroFactory {

	public static IGyro createGyro(String logPath) {
		logPath += "/Gyro";
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> Pigeon2IMUBuilder.buildGyro(logPath);
			case SIMULATION -> SimulationGyroBuilder.buildGyro(logPath);
		};
	}

	public static IMUSignals createSignals(IGyro gyro) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> Pigeon2IMUBuilder.buildSignals((Pigeon2Gyro) gyro);
			case SIMULATION -> SimulationGyroBuilder.buildSignals();
		};
	}

}
