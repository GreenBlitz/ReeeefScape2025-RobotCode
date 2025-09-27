package frc.robot.statemachine.AStarFinder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.statemachine.RobotState;

import java.util.LinkedList;
import java.util.Optional;
import java.util.function.BiFunction;

public class AStarFinder {

	private static BiFunction<RobotState, RobotState, Double> hurristicSupplier = (currentState, targetState) -> 1.0;

	public static Command findSequence(Pair<RobotState, RobotState> states, Robot robot) {
		StateNode startingState = new StateNode(states.getFirst(), 0, hurristicSupplier.apply(states.getFirst(), states.getSecond()), null);
		RobotState targetState = states.getSecond();

		StateNode currentState = startingState;

		LinkedList<StateNode> checkedStates = new LinkedList<>();
		checkedStates.add(startingState);

		while (!currentState.getState().equals(targetState)) {
			StateNode temp = checkedStates.get(0);

			for (int i = 1; i < checkedStates.size(); i++) {
				if (temp.getFCost() > checkedStates.get(i).getFCost()) {
					temp = checkedStates.get(i);
				}
			}

			currentState = temp;

			RobotState[] neighbors = currentState.getState().getNeighbors();
			for (int i = 0; i < neighbors.length; i++) {
				if (neighbors[i].equals(targetState)) {
					currentState = new StateNode(targetState, Integer.MAX_VALUE, 0, currentState);
					break;
				}

				StateNode currentNeighbor;
				if (!checkForState(checkedStates, neighbors[i]).equals(Optional.empty())) {
					currentNeighbor = checkForState(checkedStates, neighbors[i]).get();
					if (currentNeighbor.getGCost() > currentState.getGCost() + 1) {
						currentNeighbor.setParent(currentState);
					}
				} else {
					currentNeighbor = new StateNode(
						neighbors[i],
						currentState.getGCost() + 1,
						hurristicSupplier.apply(neighbors[i], targetState),
						currentState
					);
					checkedStates.add(currentNeighbor);
				}
			}
		}

		Command path = robot.getRobotCommander().setState(targetState);

		while (!currentState.getParent().equals(currentState)) {
			currentState = currentState.getParent();
			path = path.beforeStarting(robot.getRobotCommander().setState(currentState.getState()));
		}

		return path;
	}

	private static Optional<StateNode> checkForState(LinkedList<StateNode> checkedStates, RobotState current) {
		for (int i = 0; i < checkedStates.size(); i++) {
			if (checkedStates.get(i).getState().equals(current)) {
				return Optional.of(checkedStates.get(i));
			}
		}
		return Optional.empty();
	}

}
