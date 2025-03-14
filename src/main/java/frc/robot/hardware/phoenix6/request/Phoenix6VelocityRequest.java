package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.VelocityRequest;

import java.util.function.Consumer;

public class Phoenix6VelocityRequest extends Phoenix6FeedForwardRequest implements VelocityRequest {

	private final Consumer<Rotation2d> setAcceleration;
	private Rotation2d acceleration;

	Phoenix6VelocityRequest(
		Rotation2d defaultSetPoint,
		ControlRequest controlRequest,
		Consumer<Rotation2d> setSetPoint,
		Consumer<Double> setFeedForward,
		Consumer<Rotation2d> setAcceleration,
		double defaultArbitraryFeedForward
	) {
		super(defaultSetPoint, controlRequest, setSetPoint, setFeedForward, defaultArbitraryFeedForward);
		this.setAcceleration = setAcceleration;
	}

	@Override
	public Phoenix6VelocityRequest withSetPoint(Rotation2d setPoint) {
		super.withSetPoint(setPoint);
		return this;
	}

	@Override
	public VelocityRequest withAcceleration(Rotation2d acceleration) {
		this.acceleration = acceleration;
		setAcceleration.accept(acceleration);
		return this;
	}

	@Override
	public Rotation2d getAcceleration() {
		return acceleration;
	}

}
