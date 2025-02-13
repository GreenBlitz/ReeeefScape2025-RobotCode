package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.IDynamicRequest;
import frc.robot.hardware.interfaces.IRequest;

import java.util.function.Consumer;

public class DynamicPhoenix6Request extends Phoenix6FeedForwardRequest implements IDynamicRequest<Rotation2d> {
	private Rotation2d velocity;
	private Consumer<Rotation2d> setVelocity;
	private Rotation2d acceleration;
	private Consumer<Rotation2d> setAcceleration;

	DynamicPhoenix6Request(Rotation2d defaultSetPoint, ControlRequest controlRequest, Consumer<Rotation2d> setSetPoint, Consumer<Double> setFeedForward, double defaultArbitraryFeedForward) {
		super(defaultSetPoint, controlRequest, setSetPoint, setFeedForward, defaultArbitraryFeedForward);
	}

	DynamicPhoenix6Request(Rotation2d defaultSetPoint, ControlRequest controlRequest, Consumer<Rotation2d> setSetPoint, Consumer<Double> setFeedForward) {
		super(defaultSetPoint, controlRequest, setSetPoint, setFeedForward);
	}
	@Override
	public IRequest<Rotation2d> withVelocity(Rotation2d setPoint) {
		this.setVelocity.accept(setPoint);
		this.velocity = setPoint;
		return this;
	}

	@Override
	public IRequest<Rotation2d> withAcceleration(Rotation2d setPoint) {
		this.setAcceleration.accept(acceleration);
		this.acceleration = setPoint;
		return this;
	}
}
