package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Consumer;

public class DynamicPhoenix6Request extends Phoenix6FeedForwardRequest {
    private final Consumer<Double> setVelocity;
    private double velocity;
    private final Consumer<Double> setAcceleration;
    private double acceleration;

    DynamicPhoenix6Request(Rotation2d defaultSetPoint, ControlRequest controlRequest, Consumer<Rotation2d> setSetPoint, Consumer<Double> setFeedForward, double defaultArbitraryFeedForward, Consumer<Double> setVelocity, double velocity, Consumer<Double> setAcceleration, double acceleration) {
        super(defaultSetPoint, controlRequest, setSetPoint, setFeedForward, defaultArbitraryFeedForward);
        this.setVelocity = setVelocity;
        this.velocity = velocity;
        this.setAcceleration = setAcceleration;
        this.acceleration = acceleration;
    }

    public DynamicPhoenix6Request withVelocity(double newVelocity) {
        setVelocity.accept(newVelocity);
        this.velocity = newVelocity;
        return this;
    }

    public double getVelocity() {
        return this.velocity;
    }

    public DynamicPhoenix6Request withAcceleration(double newAcceleration) {
        setAcceleration.accept(newAcceleration);
        this.acceleration = newAcceleration;
        return this;
    }

    public double getAcceleration() {
        return this.acceleration;
    }
}
