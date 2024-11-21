package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.utils.time.TimeUtils;

public class FlywheelSimulation extends MechanismSimulation {

	private final FlywheelSim flywheelSimulation;

	private Rotation2d position;

	public FlywheelSimulation(FlywheelSim flywheelSimulation, double gearRatio) {
		super(gearRatio);
		this.flywheelSimulation = flywheelSimulation;
		this.position = Rotation2d.fromDegrees(0);
	}

	@Override
	public Rotation2d getSystemPosition() {
		return position;
	}

	@Override
	public Rotation2d getSystemVelocityAnglesPerSecond() {
		return Rotation2d.fromRadians(flywheelSimulation.getAngularVelocityRadPerSec());
	}

	@Override
	public void setInputVoltage(double voltage) {
		flywheelSimulation.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		flywheelSimulation.update(TimeUtils.getCurrentCycleTimeSeconds());
		Rotation2d distanceDelta = getSystemVelocityAnglesPerSecond().times(TimeUtils.getCurrentCycleTimeSeconds());
		position = Rotation2d.fromRotations(position.getRotations() + distanceDelta.getRotations());
	}

}
