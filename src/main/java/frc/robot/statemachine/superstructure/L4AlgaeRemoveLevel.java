package frc.robot.statemachine.superstructure;

import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.endeffector.EndEffectorState;

public enum L4AlgaeRemoveLevel {

    LOW(
        ElevatorState.LOW_ALGAE_REMOVE_FROM_L4,
        ElevatorState.LOW_ALGAE_REMOVE_FROM_L4,
        ElevatorState.LOW_ALGAE_REMOVE_FROM_L4,
        ArmState.ALGAE_REMOVE_FROM_L4,
        ArmState.ALGAE_REMOVE_FROM_L4,
        ArmState.ALGAE_REMOVE_FROM_L4,
        EndEffectorState.ALGAE_INTAKE
    ),
    HIGH(
        ElevatorState.HIGH_ALGAE_REMOVE_FROM_L4,
        ElevatorState.HIGH_ALGAE_REMOVE_FROM_L4,
        ElevatorState.HIGH_ALGAE_REMOVE_FROM_L4,
        ArmState.ALGAE_REMOVE_FROM_L4,
        ArmState.ALGAE_REMOVE_FROM_L4,
        ArmState.ALGAE_REMOVE_FROM_L4,
        EndEffectorState.ALGAE_INTAKE
    );

    private final ElevatorState elevatorState;
    private final ElevatorState preElevatorState;
    private final ElevatorState postElevatorState;
    private final ArmState armState;
    private final ArmState preArmState;
    private final ArmState postArmState;
    private final EndEffectorState endEffectorState;

    L4AlgaeRemoveLevel(
        ElevatorState elevatorState,
        ElevatorState preElevatorState,
        ElevatorState postElevatorState,
        ArmState armState,
        ArmState preArmState,
        ArmState postArmState,
        EndEffectorState endEffectorState
    ) {
        this.elevatorState = elevatorState;
        this.preElevatorState = preElevatorState;
        this.postElevatorState = postElevatorState;
        this.armState = armState;
        this.preArmState = preArmState;
        this.postArmState = postArmState;
        this.endEffectorState = endEffectorState;
    }

    public ElevatorState getElevatorState() {
        return elevatorState;
    }

    public ElevatorState getPreElevatorState() {
        return preElevatorState;
    }

    public ElevatorState getPostElevatorState() {
        return postElevatorState;
    }

    public ArmState getArmState() {
        return armState;
    }

    public ArmState getPreArmState() {
        return preArmState;
    }

    public ArmState getPostArmState() {
        return postArmState;
    }

    public EndEffectorState getEndEffectorState() {
        return endEffectorState;
    }

}
