package frc.robot.statemachine.aStarFinder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.statemachine.RobotCommander;
import frc.robot.statemachine.RobotState;

import java.util.LinkedList;
import java.util.Optional;
import java.util.function.BiFunction;

public class AStarFinder {

	private static BiFunction<RobotState, RobotState, Double> hurristicSupplier = (currentState, targetState) -> 0.0;

	public static Command findSequence(Pair<RobotState, RobotState> states, RobotCommander robotCommander) {
		StateNode startingState = new StateNode(states.getFirst(), 0, hurristicSupplier.apply(states.getFirst(), states.getSecond()), null);
		RobotState targetState = states.getSecond();

		StateNode currentState = startingState;

		LinkedList<StateNode> checkedStates = new LinkedList<>();
		LinkedList<StateNode> uncheckedNeighbors = new LinkedList<>();
		checkedStates.add(startingState);

		for (int i = 0; i < startingState.getState().getNeighbors().length; i++) {
			uncheckedNeighbors.add(
				new StateNode(
					startingState.getState().getNeighbors()[i],
					1,
					hurristicSupplier.apply(startingState.getState().getNeighbors()[i], targetState),
					startingState
				)
			);
		}

		while (!currentState.getState().equals(targetState)) {
			currentState = findNextStateToCheck(uncheckedNeighbors);
			checkedStates.add(currentState);

			RobotState[] neighbors = currentState.getState().getNeighbors();
			for (int i = 0; i < neighbors.length; i++) {
				if (neighbors[i].equals(targetState)) {
					currentState = new StateNode(targetState, currentState.getGCost() + 1, 0, currentState);
					break;
				}

				if (checkIfStateWasVisited(checkedStates, neighbors[i]).equals(Optional.empty())) {
					uncheckedNeighbors.add(
						new StateNode(
							neighbors[i],
							currentState.getGCost() + 1,
							hurristicSupplier.apply(neighbors[i], targetState),
							currentState
						)
					);
				}
			}
		}

		return buildPath(currentState, startingState, robotCommander);
	}

	private static Optional<StateNode> checkIfStateWasVisited(LinkedList<StateNode> checkedStates, RobotState current) {
		for (int i = 0; i < checkedStates.size(); i++) {
			if (checkedStates.get(i).getState().equals(current)) {
				return Optional.of(checkedStates.get(i));
			}
		}
		return Optional.empty();
	}

	private static StateNode findNextStateToCheck(LinkedList<StateNode> uncheckedNeighbors) {
		StateNode temp = uncheckedNeighbors.get(0);
		int tempIndex = 0;

		for (int i = 1; i < uncheckedNeighbors.size(); i++) {
			if (temp.getFCost() > uncheckedNeighbors.get(i).getFCost()) {
				temp = uncheckedNeighbors.get(i);
				tempIndex = i;
			}
			if (temp.getFCost() == uncheckedNeighbors.get(i).getFCost()) {
				if (temp.getGCost() > uncheckedNeighbors.get(i).getGCost()) {
					temp = uncheckedNeighbors.get(i);
					tempIndex = i;
				}
			}
		}

		uncheckedNeighbors.remove(tempIndex);
		return temp;
	}

	private static Command buildPath(StateNode currentState, StateNode startingState, RobotCommander robotCommander) {
		Command path = Commands.none();
		while (currentState != startingState) {
			System.out.print(currentState.getState().name() + " <- ");
			path = path.beforeStarting(robotCommander.applyState(currentState.getState()));
			currentState = currentState.getParent();
		}

		return path;
	}

}
