package frc.constants.field.enums;

import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.endeffector.EndEffectorState;

public enum AlgaeRemoveLevel {

	LOW(
		ElevatorState.LOW_ALGAE_REMOVE,
		ElevatorState.FAST_LOW_SUPER_ALGAE_REMOVE,
		ElevatorState.SLOW_LOW_SUPER_ALGAE_REMOVE,
		ElevatorState.LOW_SUPER_ALGAE_REMOVE_SLOWING_POINT,
		ArmState.LOW_ALGAE_REMOVE,
		ArmState.LOW_SUPER_ALGAE_REMOVE,
		EndEffectorState.ALGAE_INTAKE
	),
	HIGH(
		ElevatorState.HIGH_ALGAE_REMOVE,
		ElevatorState.FAST_HIGH_SUPER_ALGAE_REMOVE,
		ElevatorState.SLOW_HIGH_SUPER_ALGAE_REMOVE,
		ElevatorState.HIGH_SUPER_ALGAE_REMOVE_SLOWING_POINT,
		ArmState.HIGH_ALGAE_REMOVE,
		ArmState.HIGH_SUPER_ALGAE_REMOVE,
		EndEffectorState.ALGAE_INTAKE
	);

	private final ElevatorState regularElevatorState;
	private final ElevatorState fastSuperElevatorState;
	private final ElevatorState slowSuperElevatorState;
	private final ElevatorState superSlowingElevatorState;
	private final ArmState regularArmState;
	private final ArmState superArmState;
	private final EndEffectorState endEffectorState;

	AlgaeRemoveLevel(
		ElevatorState regularElevatorState,
		ElevatorState fastSuperElevatorState,
		ElevatorState slowSuperElevatorState,
		ElevatorState superSlowingElevatorState,
		ArmState regularArmState,
		ArmState superArmState,
		EndEffectorState endEffectorState
	) {
		this.regularElevatorState = regularElevatorState;
		this.fastSuperElevatorState = fastSuperElevatorState;
		this.slowSuperElevatorState = slowSuperElevatorState;
		this.superSlowingElevatorState = superSlowingElevatorState;
		this.regularArmState = regularArmState;
		this.superArmState = superArmState;
		this.endEffectorState = endEffectorState;
	}

	public ElevatorState getRegularElevatorState() {
		return regularElevatorState;
	}

	public ElevatorState getFastSuperElevatorState() {
		return fastSuperElevatorState;
	}

	public ElevatorState getSlowSuperElevatorState() {
		return slowSuperElevatorState;
	}

	public ElevatorState getSuperSlowingElevatorState() {
		return superSlowingElevatorState;
	}

	public ArmState getRegularArmState() {
		return regularArmState;
	}

	public EndEffectorState getEndEffectorState() {
		return endEffectorState;
	}

	public ArmState getSuperArmState() {
		return superArmState;
	}

}
