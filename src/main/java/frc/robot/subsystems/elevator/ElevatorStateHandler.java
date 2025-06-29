package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.statemachine.Tolerances;

public class ElevatorStateHandler {

	private final Elevator elevator;
	private ElevatorState currentState;

	public ElevatorStateHandler(Elevator elevator) {
		this.elevator = elevator;
	}

	public ElevatorState getCurrentState() {
		return currentState;
	}

	public Command setState(ElevatorState state) {
		if (state == ElevatorState.STAY_IN_PLACE) {
			return new ParallelCommandGroup(new InstantCommand(() -> currentState = state), elevator.getCommandsBuilder().stayInPlace());
		} else {
			return new ParallelCommandGroup(
				new InstantCommand(() -> currentState = state),
				elevator.getCommandsBuilder()
					.setTargetPositionMeters(
						state.getHeightMeters(),
						state.getMaxVelocityMetersPerSecond(),
						state.getMaxAccelerationMetersPerSecondSquared()
					)
			);
		}
	}

	public boolean isAtState(ElevatorState state, double toleranceMeters) {
		return currentState == state && elevator.isAtPosition(state.getHeightMeters(), toleranceMeters);
	}

	public boolean isAtState(ElevatorState state) {
		return currentState == state && elevator.isAtPosition(state.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS);
	}

}
