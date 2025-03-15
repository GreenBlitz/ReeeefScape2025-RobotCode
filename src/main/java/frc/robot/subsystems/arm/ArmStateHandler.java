package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.statemachine.Tolerances;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ArmStateHandler {

	private final Arm arm;
	private ArmState currentState;
	private final Supplier<Double> distanceSupplier;

	public ArmStateHandler(Arm arm, Supplier<Double> distanceSupplier) {
		this.arm = arm;
		this.distanceSupplier = distanceSupplier;
	}

	public ArmState getCurrentState() {
		return currentState;
	}

	public Command setState(ArmState state) {
		if (state == ArmState.STAY_IN_PLACE) {
			return new ParallelCommandGroup(new InstantCommand(() -> currentState = state), arm.getCommandsBuilder().stayInPlace());
		} else if (state == ArmState.L4) {
			return new ParallelCommandGroup(new InstantCommand(() -> currentState = state), arm.getCommandsBuilder().moveToPosition(() -> {
				Logger.recordOutput("distance", distanceSupplier.get());
				return state.getPosition().plus(ArmConstants.L4_DISTANCE_ANGLE_MAP.get(distanceSupplier.get()));
			}, state.getMaxVelocityRotation2dPerSecond(), state.getMaxAccelerationRotation2dPerSecondSquared()));
		} else {
			return new ParallelCommandGroup(
				new InstantCommand(() -> currentState = state),
				arm.getCommandsBuilder()
					.moveToPosition(
						state.getPosition(),
						state.getMaxVelocityRotation2dPerSecond(),
						state.getMaxAccelerationRotation2dPerSecondSquared()
					)
			);
		}
	}

	public boolean isAtState(ArmState state) {
		boolean isAt = false;
		Rotation2d inter = ArmConstants.L4_DISTANCE_ANGLE_MAP.get(distanceSupplier.get());
		Rotation2d target = Rotation2d.fromDegrees(state.getPosition().getDegrees() + inter.getDegrees());
		if (state == ArmState.L4) {
			isAt = arm.isAtPosition(
				Rotation2d
					.fromDegrees(state.getPosition().getDegrees() + ArmConstants.L4_DISTANCE_ANGLE_MAP.get(distanceSupplier.get()).getDegrees()),
				Tolerances.ARM_POSITION
			);
		} else {
			isAt = arm.isAtPosition(state.getPosition(), Tolerances.ARM_POSITION);
		}
		Logger.recordOutput("arm target", target);
		Logger.recordOutput("inter", inter);
		Logger.recordOutput("isAtAmPositon", isAt);
		return isAt;
	}

}
