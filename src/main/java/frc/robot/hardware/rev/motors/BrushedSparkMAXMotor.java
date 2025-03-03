package frc.robot.hardware.rev.motors;

import com.revrobotics.spark.SparkLowLevel;
import frc.robot.hardware.mechanisms.MechanismSimulation;

public class BrushedSparkMAXMotor extends SparkMaxMotor {

	private final SparkMaxWrapper sparkMaxWrapper;

	public BrushedSparkMAXMotor(String logPath, SparkMaxWrapper motor, MechanismSimulation mechanismSimulation) {
		super(logPath, motor, mechanismSimulation);
		if (motor.getMotorType() != SparkLowLevel.MotorType.kBrushed) {
			throw new IllegalArgumentException("inserted BrushlessSparkMAXMotor to BrushedSparkMAXMotor!");
		}
		this.sparkMaxWrapper = motor;
	}

	public BrushedSparkMAXMotor(String logPath, SparkMaxWrapper motor) {
		this(logPath, motor, null);
	}

	public SparkMaxWrapper getSparkMaxWrapper() {
		return sparkMaxWrapper;
	}
}
