package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.ReefSide;
import frc.robot.statemachine.superstructure.AlgaeRemoveLevel;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

public class ButtonDriverHelper {

	private enum LeftRightTogglePlacements {

		LEFT_TOGGLE_BRANCH_PLACEMENT(
			Field.getAllianceRelative(
				new Pose2d(TOGGLE_X_AXIS, 4.55, Rotation2d.fromDegrees(0)),
				true,
				true,
				AngleTransform.INVERT
			)
		),
		RIGHT_TOGGLE_BRANCH_PLACEMENT(
			Field.getAllianceRelative(
				new Pose2d(TOGGLE_X_AXIS, 3.55, Rotation2d.fromDegrees(0)),
				true,
				true,
				AngleTransform.INVERT
			)
		),
		HIDDEN_PLACEMENT(new Pose2d(-10, -10, Rotation2d.fromDegrees(0)));

		private final Pose2d placement;

		LeftRightTogglePlacements(Pose2d placement) {
			this.placement = placement;
		}

	}

	private enum IsRemovingAlgaeTogglePlacements {
		ALGAE_TOGGLE_PLACEMENT(new Pose2d(TOGGLE_X_AXIS, 1.5, Rotation2d.fromDegrees(0))),
		HIDDEN_PLACEMENT(new Pose2d(-10, -10, Rotation2d.fromDegrees(90)));

		private final Pose2d placement;

		IsRemovingAlgaeTogglePlacements(Pose2d placement) {
			this.placement = placement;
		}
	}

	private static final double DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS = 0.45;
	private static final double SCORE_LEVEL_Y_AXIS = 6.85;
	private static final double TOGGLE_X_AXIS = 16;

	private static final Pose2d L1_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(12.22, SCORE_LEVEL_Y_AXIS, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d L2_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(13.55, SCORE_LEVEL_Y_AXIS, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d L3_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(14.88, SCORE_LEVEL_Y_AXIS, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d L4_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(16, SCORE_LEVEL_Y_AXIS, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);

	private static int chosenSideIndex;
	private static int chosenScoreLevelIndex;
	private static int leftIndex;
	private static int rightIndex;
	private static int darkLeftRightToggleIndex;
	private static int algaeRemovingIndex;
	private static int darkAlgaeRemovingToggleIndex;

	private static final Pose2d[] REEF_SIDES = {
		Field.getPointFromCertainDistance(
			FieldMath.mirror(Field.getReefSideMiddle(ReefSide.D), true, false, AngleTransform.MIRROR_X),
			DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS
		),
		Field.getPointFromCertainDistance(
			FieldMath.mirror(Field.getReefSideMiddle(ReefSide.C), true, false, AngleTransform.MIRROR_X),
			DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS
		),
		Field.getPointFromCertainDistance(
			FieldMath.mirror(Field.getReefSideMiddle(ReefSide.B), true, false, AngleTransform.MIRROR_X),
			DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS
		),
		Field.getPointFromCertainDistance(
			FieldMath.mirror(Field.getReefSideMiddle(ReefSide.A), true, false, AngleTransform.MIRROR_X),
			DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS
		),
		Field.getPointFromCertainDistance(
			FieldMath.mirror(Field.getReefSideMiddle(ReefSide.F), true, false, AngleTransform.MIRROR_X),
			DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS
		),
		Field.getPointFromCertainDistance(
			FieldMath.mirror(Field.getReefSideMiddle(ReefSide.E), true, false, AngleTransform.MIRROR_X),
			DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS
		)};

	private static final Pose2d[] SCORE_LEVEL_PLACEMENTS = {
		L1_DISPLAY_PLACEMENT,
		L2_DISPLAY_PLACEMENT,
		L3_DISPLAY_PLACEMENT,
		L4_DISPLAY_PLACEMENT};

	private static void updateChosenReefSideIndex(ReefSide reefSide) {
		chosenSideIndex = reefSide.getIndex();
	}

	private static void updateScoreLevelIndex(ScoreLevel scoreLevel) {
		chosenScoreLevelIndex = scoreLevel.ordinal();
	}

	private static void updateLeftRightToggleIndexes(Branch branch) {
		if (branch.isLeft()) {
			leftIndex = LeftRightTogglePlacements.LEFT_TOGGLE_BRANCH_PLACEMENT.ordinal();
			rightIndex = LeftRightTogglePlacements.HIDDEN_PLACEMENT.ordinal();
			darkLeftRightToggleIndex = LeftRightTogglePlacements.RIGHT_TOGGLE_BRANCH_PLACEMENT.ordinal();
		} else {
			leftIndex = LeftRightTogglePlacements.HIDDEN_PLACEMENT.ordinal();
			rightIndex = LeftRightTogglePlacements.RIGHT_TOGGLE_BRANCH_PLACEMENT.ordinal();
			darkLeftRightToggleIndex = LeftRightTogglePlacements.LEFT_TOGGLE_BRANCH_PLACEMENT.ordinal();
		}
	}

	private static void updateAlgaeRemovingToggleIndexes() {
		if (ScoringHelpers.isTakingAlgae){
			algaeRemovingIndex = IsRemovingAlgaeTogglePlacements.ALGAE_TOGGLE_PLACEMENT.ordinal();
			darkAlgaeRemovingToggleIndex = IsRemovingAlgaeTogglePlacements.HIDDEN_PLACEMENT.ordinal();
		}
		else {
			algaeRemovingIndex = IsRemovingAlgaeTogglePlacements.HIDDEN_PLACEMENT.ordinal();
			darkAlgaeRemovingToggleIndex = IsRemovingAlgaeTogglePlacements.ALGAE_TOGGLE_PLACEMENT.ordinal();
		}
	}

	private static Pose2d[] setUpDisplay() {
		updateChosenReefSideIndex(ScoringHelpers.getTargetReefSide());
		updateScoreLevelIndex(ScoringHelpers.targetScoreLevel);
		updateLeftRightToggleIndexes(ScoringHelpers.getTargetBranch());
		updateAlgaeRemovingToggleIndexes();

		Pose2d[] darkDisplay = new Pose2d[REEF_SIDES.length + SCORE_LEVEL_PLACEMENTS.length];
		int darkDisplayIndex = 0;

		darkDisplayIndex = addArrayToArrayWithoutSpecificIndex(darkDisplay, darkDisplayIndex, REEF_SIDES, chosenSideIndex);
		darkDisplayIndex = addArrayToArrayWithoutSpecificIndex(darkDisplay, darkDisplayIndex, SCORE_LEVEL_PLACEMENTS, chosenScoreLevelIndex);
		darkDisplay[darkDisplayIndex] = LeftRightTogglePlacements.values()[darkLeftRightToggleIndex].placement;
		darkDisplay[darkDisplayIndex + 1] = IsRemovingAlgaeTogglePlacements.values()[darkAlgaeRemovingToggleIndex].placement;

		return darkDisplay;
	}

	public static void log(String logPath) {
		Logger.recordOutput(logPath + "/DarkDisplay", setUpDisplay());
		Logger.recordOutput(logPath + "/ChosenReefSide", REEF_SIDES[chosenSideIndex]);
		Logger.recordOutput(logPath + "/ChosenScoreLevel", SCORE_LEVEL_PLACEMENTS[chosenScoreLevelIndex]);
		Logger.recordOutput(logPath + "/LeftToggle", LeftRightTogglePlacements.values()[leftIndex].placement);
		Logger.recordOutput(logPath + "/RightToggle", LeftRightTogglePlacements.values()[rightIndex].placement);
		Logger.recordOutput(logPath + "/IsRemovingAlgae", IsRemovingAlgaeTogglePlacements.values()[algaeRemovingIndex].placement);
	}

	public static <T> int addArrayToArrayWithoutSpecificIndex(T[] receiver, int startIndex, T[] giver, int indexToExclude) {
		for (int index = 0; index < giver.length; index++) {
			if (index != indexToExclude) {
				receiver[startIndex] = giver[index];
				startIndex++;
			}
		}
		return startIndex;
	}

}
