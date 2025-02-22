package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import java.util.function.Consumer;

public class DynamicPhoenix6Request extends Phoenix6FeedForwardRequest {
    private final Consumer<AngularVelocity> setVelocity;
    private AngularVelocity velocity;
    private final Consumer<AngularAcceleration> setAcceleration;
    private AngularAcceleration acceleration;

    DynamicPhoenix6Request(
            Rotation2d defaultSetPoint,
            ControlRequest controlRequest,
            Consumer<Rotation2d> setSetPoint,
            Consumer<Double> setFeedForward,
            double defaultArbitraryFeedForward,
            Consumer<AngularVelocity> setVelocity,
            AngularVelocity velocity,
            Consumer<AngularAcceleration> setAcceleration,
            AngularAcceleration acceleration
    ) {
        super(defaultSetPoint, controlRequest, setSetPoint, setFeedForward, defaultArbitraryFeedForward);
        this.setVelocity = setVelocity;
        this.velocity = velocity;
        this.setAcceleration = setAcceleration;
        this.acceleration = acceleration;
    }

    public DynamicPhoenix6Request withVelocity(AngularVelocity newVelocity) {
        setVelocity.accept(newVelocity);
        this.velocity = newVelocity;
        return this;
    }

    public AngularVelocity getVelocity() {
        return this.velocity;
    }

    public DynamicPhoenix6Request withAcceleration(AngularAcceleration newAcceleration) {
        setAcceleration.accept(newAcceleration);
        this.acceleration = newAcceleration;
        return this;
    }

    public AngularAcceleration getAcceleration() {
        return this.acceleration;
    }
}
