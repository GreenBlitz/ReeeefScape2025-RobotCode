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
import frc.robot.statemachine.superstructure.ScoreLevel;

import java.util.HashMap;

public class ScoringPathsHelper {

	public static HashMap<Branch, PathPlannerPath> L4_AUTO_SCORE_PATHS;
	public static HashMap<Branch, PathPlannerPath> L2_L3_AUTO_SCORE_PATHS;

	public static void generateAutoScorePathsHashMaps(Robot robot) {
		generateL4AutoScorePaths(robot);
		generateL2L3AutoScorePaths(robot);
	}

	private static void generateL4AutoScorePaths(Robot robot) {
		L4_AUTO_SCORE_PATHS = generateAllAutoScorePaths(ScoreLevel.L4, robot);
	}

	private static void generateL2L3AutoScorePaths(Robot robot) {
		L2_L3_AUTO_SCORE_PATHS = generateAllAutoScorePaths(ScoreLevel.L3, robot);
	}

	private static HashMap<Branch, PathPlannerPath> generateAllAutoScorePaths(ScoreLevel scoreLevel, Robot robot) {
		HashMap<Branch, PathPlannerPath> branchToPathMap = new HashMap<>();

		Branch[] branches = Branch.values();
		for (Branch branch : branches) {
			branchToPathMap.put(branch, generatePathToTargetBranch(branch, scoreLevel, robot));
		}
		return branchToPathMap;
	}


	public static PathPlannerPath getPathByBranch(Branch branch, ScoreLevel scoreLevel) {
		return switch (scoreLevel) {
			case L1, L2, L3 -> L2_L3_AUTO_SCORE_PATHS.get(branch);
			case L4 -> L4_AUTO_SCORE_PATHS.get(branch);
		};
	}

	private static PathPlannerPath generatePathToTargetBranch(Branch branch, ScoreLevel scoreLevel, Robot robot) {
		PathConstraints pathConstraints = scoreLevel == ScoreLevel.L4
			? AutonomousConstants.getRealTimeConstraintsForL4Path(robot.getSwerve())
			: AutonomousConstants.getRealTimeConstraintsForL2L3Path(robot.getSwerve());
		double distanceToBranchForStartingPath = scoreLevel == ScoreLevel.L4
			? StateMachineConstants.L4_DISTANCE_TO_BRANCH_FOR_STARTING_PATH
			: StateMachineConstants.L2_L3_DISTANCE_TO_BRANCH_FOR_STARTING_PATH;
		double idealStartingStateVelocity = scoreLevel == ScoreLevel.L4
			? StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND
			: StateMachineConstants.L2_L3_IDEAL_STARTING_STATE_VELOCITY_METERS_PER_SECOND;

		return new PathPlannerPath(
			PathPlannerPath.waypointsFromPoses(
				ScoringHelpers.getRobotBranchScoringPose(branch, distanceToBranchForStartingPath, false),
				ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS, false)
			),
			pathConstraints,
			new IdealStartingState(idealStartingStateVelocity, Field.getReefSideMiddle(branch.getReefSide(), false).getRotation()),
			new GoalEndState(0, Field.getReefSideMiddle(branch.getReefSide(), false).getRotation())
		);
	}


}
