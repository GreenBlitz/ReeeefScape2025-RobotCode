package frc.robot.statemachine.superstructure;

import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.endeffector.EndEffectorState;

public enum AlgaeRemoveLevel {

	LOW(ElevatorState.LOW_ALGAE_REMOVE, ArmState.LOW_ALGAE_REMOVE),
	HIGH(ElevatorState.HIGH_ALGAE_REMOVE, ArmState.HIGH_ALGAE_REMOVE);

	private final ElevatorState elevatorState;
	private final ArmState armState;

	AlgaeRemoveLevel(ElevatorState elevatorState, ArmState armState) {
		this.elevatorState = elevatorState;
		this.armState = armState;
	}

	public ElevatorState getElevatorState() {
		return elevatorState;
	}

	public ArmState getArmState() {
		return armState;
	}


}
