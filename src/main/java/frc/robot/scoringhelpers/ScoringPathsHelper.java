package frc.robot.scoringhelpers;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.ReefSide;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.statemachine.StateMachineConstants;

import java.util.HashMap;

public class ScoringPathsHelper {

	private static final HashMap<Branch, PathPlannerPath> BRANCH_PATH_PLANNER_PATH_HASH_MAP = generateAllBranchPaths();
	private static HashMap<ReefSide, PathPlannerPath> REEF_SIDE_PATH_PLANNER_PATH_HASH_MAP;

	public static void generateAlgaeRemovePaths(Robot robot) {
		REEF_SIDE_PATH_PLANNER_PATH_HASH_MAP = generateAllReefSidePaths(robot);
	}

	private static HashMap<Branch, PathPlannerPath> generateAllBranchPaths() {
		HashMap<Branch, PathPlannerPath> branchToPathMap = new HashMap<>();

		Branch[] branches = Branch.values();
		for (Branch branch : branches) {
			branchToPathMap.put(branch, generatePathToTargetBranch(branch));
		}
		return branchToPathMap;
	}

	private static HashMap<ReefSide, PathPlannerPath> generateAllReefSidePaths(Robot robot) {
		HashMap<ReefSide, PathPlannerPath> reefSideToPathMap = new HashMap<>();

		ReefSide[] reefSides = ReefSide.values();
		for (ReefSide reefSide : reefSides) {
			reefSideToPathMap.put(reefSide, generatePathToAlgaeRemove(reefSide, robot));
		}
		return reefSideToPathMap;
	}


	public static PathPlannerPath getPathByBranch(Branch branch) {
		return BRANCH_PATH_PLANNER_PATH_HASH_MAP.get(branch);
	}

	public static PathPlannerPath getPathByReefSide(ReefSide reefSide) {
		return REEF_SIDE_PATH_PLANNER_PATH_HASH_MAP.get(reefSide);
	}

	private static PathPlannerPath generatePathToTargetBranch(Branch branch) {
		return new PathPlannerPath(
			PathPlannerPath.waypointsFromPoses(
				ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.DISTANCE_TO_BRANCH_FOR_STARTING_PATH, false),
				ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS, false)
			),
			new PathConstraints(
				StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND,
				StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_METERS_PER_SECOND_SQUARED,
				StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND.getRadians(),
				StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND_SQUARED.getRadians()
			),
			new IdealStartingState(
				StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND,
				Field.getReefSideMiddle(branch.getReefSide(), false).getRotation()
			),
			new GoalEndState(0, Field.getReefSideMiddle(branch.getReefSide(), false).getRotation())
		);
	}

	private static PathPlannerPath generatePathToAlgaeRemove(ReefSide reefSide, Robot robot) {
		return new PathPlannerPath(
			PathPlannerPath.waypointsFromPoses(
				ScoringHelpers.getAlgaeRemovePose(reefSide, StateMachineConstants.DISTANCE_TO_REEF_SIDE_FOR_STARTING_PATH, false),
				ScoringHelpers.getAlgaeRemovePose(reefSide, StateMachineConstants.ROBOT_ALGAE_REMOVE_DISTANCE_FROM_REEF_METERS, false)
			),
			AutonomousConstants.getRealTimeConstraints(robot.getSwerve()),
			new IdealStartingState(
				StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND,
				Field.getReefSideMiddle(reefSide, false).getRotation()
			),
			new GoalEndState(0, Field.getReefSideMiddle(reefSide, false).getRotation())
		);
	}


}
