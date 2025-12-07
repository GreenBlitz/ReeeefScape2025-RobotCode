package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.statemachine.superstructure.TargetChecks;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ElevatorStateHandler {

	private final Elevator elevator;
	private ElevatorState currentState;
	public static LoggedNetworkNumber tunableHeight = new LoggedNetworkNumber("/Tuning/elevatorMeters", 0.0);

	public ElevatorStateHandler(Elevator elevator) {
		this.elevator = elevator;
	}

	public ElevatorState getCurrentState() {
		return currentState;
	}

	public Command setState(ElevatorState state) {
		if (state == ElevatorState.STAY_IN_PLACE) {
			return new ParallelCommandGroup(new InstantCommand(() -> currentState = state), elevator.getCommandsBuilder().stayInPlace());
		} else if (state == ElevatorState.CALIBRATION) {
			return new ParallelCommandGroup(
				new InstantCommand(() -> currentState = state),
				elevator.getCommandsBuilder()
					.setTargetPositionMeters(
						() -> tunableHeight.get(),
						state.getMaxVelocityMetersPerSecond(),
						state.getMaxAccelerationMetersPerSecondSquared()
					)
			);
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
		return currentState == state && elevator.isAtPosition(state.getHeightMeters(), TargetChecks.ELEVATOR_HEIGHT_TOLERANCE_METERS);
	}

}
