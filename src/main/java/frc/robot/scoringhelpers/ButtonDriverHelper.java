package frc.robot.scoringhelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.ReefSide;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.utils.math.AngleTransform;
import org.littletonrobotics.junction.Logger;

public class ButtonDriverHelper {

	private enum LeftRightTogglePlacements {

		LEFT_TOGGLE_PLACEMENT(
			Field.getAllianceRelative(new Pose2d(LEFT_RIGHT_TOGGLE_X_AXIS, 4.55, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT)
		),
		RIGHT_TOGGLE_PLACEMENT(
			Field.getAllianceRelative(new Pose2d(LEFT_RIGHT_TOGGLE_X_AXIS, 3.55, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT)
		),
		HIDDEN_PLACEMENT(new Pose2d(-10, -10, Rotation2d.fromDegrees(90)));

		private final Pose2d placement;

		LeftRightTogglePlacements(Pose2d placement) {
			this.placement = placement;
		}

	}

	private static final double DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS = 0.45;
	private static final double SCORE_LEVEL_Y_AXIS = 6.85;
	private static final double LEFT_RIGHT_TOGGLE_X_AXIS = 7.22;

	private static final Pose2d L1_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(3.22, SCORE_LEVEL_Y_AXIS, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d L2_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(4.55, SCORE_LEVEL_Y_AXIS, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d L3_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(5.88, SCORE_LEVEL_Y_AXIS, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);
	private static final Pose2d L4_DISPLAY_PLACEMENT = Field
		.getAllianceRelative(new Pose2d(7.22, SCORE_LEVEL_Y_AXIS, Rotation2d.fromDegrees(0)), true, true, AngleTransform.INVERT);

	private static int chosenSideIndex = 0;
	private static int chosenScoreLevelIndex = 0;
	private static int leftIndex = 0;
	private static int rightIndex = 1;
	private static int darkToggleIndex = 2;

	private static final Pose2d[] REEF_SIDES = {
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.A), DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.B), DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.C), DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.D), DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.E), DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS),
		getPointFromCertainDistance(Field.getReefSideMiddle(ReefSide.F), DISTANCE_FROM_REEF_FOR_SIDE_HIGHLIGHTING_METERS)};

	private static final Pose2d[] SCORE_LEVEL_PLACEMENTS = {
		L1_DISPLAY_PLACEMENT,
		L2_DISPLAY_PLACEMENT,
		L3_DISPLAY_PLACEMENT,
		L4_DISPLAY_PLACEMENT};

	public static void updateChosenReefSideIndex(ReefSide reefSide) {
		chosenSideIndex = reefSide.getIndex();
	}

	public static void updateScoreLevelIndex(ScoreLevel scoreLevel) {
		chosenScoreLevelIndex = scoreLevel.ordinal();
	}

	public static void updateLeftRightToggleIndexes(Branch branch) {
		if (branch.isLeft()) {
			leftIndex = LeftRightTogglePlacements.LEFT_TOGGLE_PLACEMENT.ordinal();
			rightIndex = LeftRightTogglePlacements.HIDDEN_PLACEMENT.ordinal();
			darkToggleIndex = LeftRightTogglePlacements.RIGHT_TOGGLE_PLACEMENT.ordinal();
		} else {
			leftIndex = LeftRightTogglePlacements.HIDDEN_PLACEMENT.ordinal();
			rightIndex = LeftRightTogglePlacements.RIGHT_TOGGLE_PLACEMENT.ordinal();
			darkToggleIndex = LeftRightTogglePlacements.LEFT_TOGGLE_PLACEMENT.ordinal();
		}
	}

	public static Pose2d[] getDarkPlacementsDisplay() {
		updateChosenReefSideIndex(ScoringHelpers.getTargetReefSide());
		updateScoreLevelIndex(ScoringHelpers.targetScoreLevel);
		updateLeftRightToggleIndexes(ScoringHelpers.getTargetBranch());

		Pose2d[] darkDisplay = new Pose2d[REEF_SIDES.length + SCORE_LEVEL_PLACEMENTS.length - 1];
		int darkDisplayIndex = 0;

		for (int index = 0; index < REEF_SIDES.length; index++) {
			if (index != chosenSideIndex) {
				darkDisplay[darkDisplayIndex] = REEF_SIDES[index];
				darkDisplayIndex++;
			}
		}
		for (int index = 0; index < SCORE_LEVEL_PLACEMENTS.length; index++) {
			if (index != chosenScoreLevelIndex) {
				darkDisplay[darkDisplayIndex] = SCORE_LEVEL_PLACEMENTS[index];
				darkDisplayIndex++;
			}
		}
		darkDisplay[darkDisplayIndex] = LeftRightTogglePlacements.values()[darkToggleIndex].placement;

		return darkDisplay;
	}

	public static Pose2d getPointFromCertainDistance(Pose2d point, double distantInMeters) {
		return new Pose2d(
			point.getX() - point.getRotation().getCos() * distantInMeters,
			point.getY() - point.getRotation().getSin() * distantInMeters,
			point.getRotation()
		);
	}

	public static void log(String logPath) {
		Logger.recordOutput(logPath + "/DarkDisplay", getDarkPlacementsDisplay());
		Logger.recordOutput(logPath + "/ChosenReefSide", REEF_SIDES[chosenSideIndex]);
		Logger.recordOutput(logPath + "/ChosenScoreLevel", SCORE_LEVEL_PLACEMENTS[chosenScoreLevelIndex]);
		Logger.recordOutput(logPath + "/LeftToggle", LeftRightTogglePlacements.values()[leftIndex].placement);
		Logger.recordOutput(logPath + "/RightToggle", LeftRightTogglePlacements.values()[rightIndex].placement);
	}

}
