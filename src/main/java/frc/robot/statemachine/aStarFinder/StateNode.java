package frc.robot.statemachine.aStarFinder;

import frc.robot.statemachine.RobotState;

public class StateNode {

	private RobotState state;
	private double gCost;
	private double hCost;
	private StateNode parent;

	public StateNode(RobotState state, double gCost, double hCost, StateNode parent) {
		this.state = state;
		this.gCost = gCost;
		this.hCost = hCost;
		this.parent = parent;
	}

	public RobotState getState() {
		return state;
	}

	public double getGCost() {
		return gCost;
	}

	public double getHCost() {
		return hCost;
	}

	public double getFCost() {
		return gCost + hCost;
	}

	public StateNode getParent() {
		return parent;
	}

	public void setGCost(double gCost) {
		this.gCost = gCost;
	}

	public void setParent(StateNode parent) {
		this.parent = parent;
		setGCost(parent.getGCost() + 1);
	}

}
