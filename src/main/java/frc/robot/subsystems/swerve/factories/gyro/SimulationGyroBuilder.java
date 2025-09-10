package frc.robot.subsystems.swerve.factories.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.empties.EmptyGyro;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.subsystems.swerve.IMUSignals;
import frc.utils.TimedValue;
import frc.utils.AngleUnit;
import frc.utils.time.TimeUtil;

class SimulationGyroBuilder {

	static IGyro buildGyro(String logPath) {
		return new EmptyGyro(logPath);
	}

	private static AngleSignal buildEmptyAngleSignal(String name) {
		return new AngleSignal(name, AngleUnit.DEGREES) {

			@Override
			protected void updateValue(TimedValue<Rotation2d> timedValue) {
				timedValue.setValue(new Rotation2d());
				timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
			}

		};
	}

	private static DoubleSignal buildEmptyDoubleSignal(String name) {
		return new DoubleSignal(name) {

			@Override
			protected void updateValue(TimedValue<Double> timedValue) {
				timedValue.setValue(0.0);
				timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds());
			}

		};
	}

	static IMUSignals buildSignals() {
		return new IMUSignals(
			buildEmptyAngleSignal("yaw"),
			buildEmptyAngleSignal("roll"),
			buildEmptyAngleSignal("pitch"),
			buildEmptyAngleSignal("angularVelocityX"),
			buildEmptyAngleSignal("angularVelocityY"),
			buildEmptyAngleSignal("angularVelocityZ"),
			buildEmptyDoubleSignal("accelerationX"),
			buildEmptyDoubleSignal("accelerationY"),
			buildEmptyDoubleSignal("accelerationZ")
		);
	}

}
