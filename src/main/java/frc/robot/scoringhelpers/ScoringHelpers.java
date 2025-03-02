package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.AlgaeRemoveLevel;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.CoralStationSlot;
import frc.constants.field.enums.CoralStation;
import frc.constants.field.enums.ReefSide;
import frc.constants.field.enums.*;
import frc.robot.Robot;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.utils.math.AngleTransform;
import frc.utils.pose.Side;
import org.littletonrobotics.junction.Logger;

public class ScoringHelpers {

	public static final Translation2d END_EFFECTOR_OFFSET_FROM_MID_ROBOT = new Translation2d(0, 0.014);
	public static final Rotation2d HEADING_FOR_NET = Rotation2d.fromDegrees(0);
	public static final Rotation2d HEADING_FOR_CAGE = Rotation2d.fromDegrees(180);
	private static final Pose2d PROCESSOR_SCORING_POSE = new Pose2d(6, 0.7, Field.getProcessor().getRotation());

	public static ScoreLevel targetScoreLevel = ScoreLevel.L4;

	private static boolean isFarReefHalf = false;
	private static Side targetSideForReef = Side.MIDDLE;
	private static boolean isLeftBranch = false;
	private static CoralStation latestWantedCoralStation = CoralStation.LEFT;
	private static CoralStationSlot latestWantedCoralStationSlot = CoralStationSlot.L1;
	private static Cage latestWantedCage = Cage.FIELD_WALL;


	public static ReefSide getTargetReefSide() {
		return ReefSide.getReefSideBySideAndFar(targetSideForReef, isFarReefHalf);
	}

	public static Branch getTargetBranch() {
		return Branch.getBranchByReefSideAndSide(getTargetReefSide(), isLeftBranch);
	}

	public static CoralStation getTargetCoralStation(Robot robot) {
		Translation2d robotTranslation = robot.getPoseEstimator().getEstimatedPose().getTranslation();
		Translation2d leftCoralStationTranslation = Field.getCoralStationMiddle(CoralStation.LEFT).getTranslation();
		Translation2d rightCoralStationTranslation = Field.getCoralStationMiddle(CoralStation.RIGHT).getTranslation();
		if (robotTranslation.getDistance(leftCoralStationTranslation) < robotTranslation.getDistance(rightCoralStationTranslation)) {
			latestWantedCoralStation = CoralStation.LEFT;
		} else {
			latestWantedCoralStation = CoralStation.RIGHT;
		}
		return latestWantedCoralStation;
	}

	public static CoralStationSlot getTargetCoralStationSlot(Robot robot) {
		Translation2d robotTranslation = robot.getPoseEstimator().getEstimatedPose().getTranslation();
		if (getTargetCoralStation(robot) == CoralStation.RIGHT) {
			latestWantedCoralStationSlot = getClosestCoralStationSlot(robotTranslation, CoralStationSlot.R2, CoralStationSlot.R8);
		} else {
			latestWantedCoralStationSlot = getClosestCoralStationSlot(robotTranslation, CoralStationSlot.L2, CoralStationSlot.L8);
		}
		return latestWantedCoralStationSlot;
	}

	public static Cage getTargetCage(Robot robot) {
		Translation2d robotTranslation = robot.getPoseEstimator().getEstimatedPose().getTranslation();
		latestWantedCage = getClosestCage(robotTranslation, Cage.FIELD_WALL, Cage.MIDDLE, Cage.FIELD_CENTER);
		return latestWantedCage;
	}

	public static AlgaeRemoveLevel getAlgaeRemoveLevel() {
		if (isFarReefHalf) {
			return switch (targetSideForReef) {
				case LEFT, RIGHT -> AlgaeRemoveLevel.HIGH;
				case MIDDLE -> AlgaeRemoveLevel.LOW;
			};
		} else {
			return switch (targetSideForReef) {
				case LEFT, RIGHT -> AlgaeRemoveLevel.LOW;
				case MIDDLE -> AlgaeRemoveLevel.HIGH;
			};
		}
	}

	public static void toggleIsFarReefHalf() {
		isFarReefHalf = !isFarReefHalf;
	}

	public static void toggleIsLeftBranch() {
		isLeftBranch = !isLeftBranch;
	}

	public static void setTargetSideForReef(Side side) {
		targetSideForReef = side;
	}


	public static Pose2d getRobotBranchScoringPose(Branch branch, double distanceFromBranchMeters, boolean isAllianceRelative) {
		Translation2d branchTranslation = Field.getCoralPlacement(branch, isAllianceRelative);
		Rotation2d targetRobotAngle = Field.getReefSideMiddle(branch.getReefSide(), isAllianceRelative).getRotation();
		Translation2d differenceTranslation = new Translation2d(distanceFromBranchMeters, targetRobotAngle);
		Translation2d endeffectorOffsetDifference = END_EFFECTOR_OFFSET_FROM_MID_ROBOT.rotateBy(targetRobotAngle);
		return new Pose2d(branchTranslation.minus(differenceTranslation).minus(endeffectorOffsetDifference), targetRobotAngle);
	}

	public static void setClosetReefSideTarget(Pose2d robotPose) {
		ReefSide closetReefSide = getNearestReefSide(robotPose);
		targetSideForReef = closetReefSide.getSide();
		isFarReefHalf = closetReefSide.isFar();
	}

	public static ReefSide getNearestReefSide(Pose2d robotPose) {
		ReefSide[] reefSides = ReefSide.values();
		ReefSide closetSide = reefSides[0];

		double minDistance = robotPose.getTranslation().getDistance(Field.getReefSideMiddle(closetSide).getTranslation());
		for (int i = 1; i < reefSides.length; i++) {
			double distanceFromBranch = robotPose.getTranslation().getDistance(Field.getReefSideMiddle(reefSides[i]).getTranslation());
			if (distanceFromBranch < minDistance) {
				closetSide = reefSides[i];
				minDistance = distanceFromBranch;
			}
		}

		return closetSide;
	}

	public static Pose2d getRobotBranchScoringPose(Branch branch, double distanceFromBranchMeters) {
		return getRobotBranchScoringPose(branch, distanceFromBranchMeters, true);
	}

	public static Pose2d getRobotRelativeAlgaeRemovePose(ReefSide side, double distanceFromReefMeters) {
		Translation2d reefMiddleTranslation = Field.getReefSideMiddle(side).getTranslation();
		Rotation2d targetRobotAngle = Field.getReefSideMiddle(side).getRotation();
		Translation2d differenceTranslation = new Translation2d(distanceFromReefMeters, targetRobotAngle);
		return new Pose2d(reefMiddleTranslation.minus(differenceTranslation), targetRobotAngle);
	}

	public static Pose2d getAllianceRelativeProcessorScoringPose() {
		return Field.getAllianceRelative(PROCESSOR_SCORING_POSE, true, true, AngleTransform.KEEP);
	}

	public static void log(String logPath) {
		Logger.recordOutput(logPath + "/TargetBranch", getTargetBranch());
		Logger.recordOutput(logPath + "/TargetReefSide", getTargetReefSide());
		Logger.recordOutput(logPath + "/TargetCoralStation", latestWantedCoralStation);
		Logger.recordOutput(logPath + "/TargetCoralStationSlot", latestWantedCoralStationSlot);
		Logger.recordOutput(logPath + "/TargetScoreLevel", targetScoreLevel);
		Logger.recordOutput(logPath + "/TargetCage", latestWantedCage);
	}

	private static CoralStationSlot getClosestCoralStationSlot(Translation2d robotTranslation, CoralStationSlot... slots) {
		double[] distances = new double[slots.length];
		int closestSlotIndex = 0;

		for (int i = 0; i < slots.length; i++) {
			distances[i] = robotTranslation.getDistance(Field.getCoralStationSlot(slots[i]).getTranslation());
		}
		for (int i = 1; i < distances.length; i++) {
			if (distances[i] < distances[closestSlotIndex]) {
				closestSlotIndex = i;
			}
		}
		return slots[closestSlotIndex];
	}

	public static Pose2d getIntakePose(CoralStationSlot coralStationSlot) {
		Pose2d coralStationSlotPose = Field.getCoralStationSlot(coralStationSlot);
		Translation2d rotatedEndEffectorOffset = ScoringHelpers.END_EFFECTOR_OFFSET_FROM_MID_ROBOT.rotateBy(coralStationSlotPose.getRotation());

		return new Pose2d(coralStationSlotPose.getTranslation().plus(rotatedEndEffectorOffset), coralStationSlotPose.getRotation());
	}

	private static Cage getClosestCage(Translation2d robotTranslation, Cage... cages) {
		double[] distances = new double[cages.length];
		int closestSlotIndex = 0;

		for (int i = 0; i < cages.length; i++) {
			distances[i] = robotTranslation.getDistance(Field.getCage(cages[i]).getTranslation());
		}
		for (int i = 1; i < distances.length; i++) {
			if (distances[i] < distances[closestSlotIndex]) {
				closestSlotIndex = i;
			}
		}
		return cages[closestSlotIndex];
	}

	public static void reset() {
		isFarReefHalf = false;
		isLeftBranch = false;
		targetSideForReef = Side.MIDDLE;
	}

}
