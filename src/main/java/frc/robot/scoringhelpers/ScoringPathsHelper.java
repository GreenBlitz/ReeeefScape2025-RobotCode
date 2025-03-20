package frc.robot.scoringhelpers;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.statemachine.StateMachineConstants;

import java.util.HashMap;

public class ScoringPathsHelper {

	private static HashMap<Branch, PathPlannerPath> BRANCH_PATH_PLANNER_PATH_HASH_MAP(Robot robot){
		 return generateAllPaths(robot);
	}

	private static HashMap<Branch, PathPlannerPath> generateAllPaths(Robot robot) {
		HashMap<Branch, PathPlannerPath> branchToPathMap = new HashMap<>();

		Branch[] branches = Branch.values();
		for (Branch branch : branches) {
			branchToPathMap.put(branch, generatePathToTargetBranch(branch, robot));
		}
		return branchToPathMap;
	}


	public static PathPlannerPath getPathByBranch(Branch branch, Robot robot) {
		return BRANCH_PATH_PLANNER_PATH_HASH_MAP(robot).get(branch);
	}

	private static PathPlannerPath generatePathToTargetBranch(Branch branch, Robot robot) {
		return new PathPlannerPath(
			PathPlannerPath.waypointsFromPoses(
				ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.DISTANCE_TO_BRANCH_FOR_STARTING_PATH, false),
				ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS, false)
			),
			AutonomousConstants.getRealTimeConstraints(robot.getSwerve()),
//			new PathConstraints(
//				StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND,
//				StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_METERS_PER_SECOND_SQUARED,
//				StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND.getRadians(),
//				StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND_SQUARED.getRadians()
//			),
			new IdealStartingState(
				StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND,
				Field.getReefSideMiddle(branch.getReefSide(), false).getRotation()
			),
			new GoalEndState(0, Field.getReefSideMiddle(branch.getReefSide(), false).getRotation())
		);
	}


}
